/*
 * 蝴蝶板灯控应用 — BLE 蓝牙控制模块实现
 *
 * 见 app_ble.h 的协议说明。
 *
 * 【NimBLE 回调与 FreeRTOS 栈】
 *
 * NimBLE 的 GATT 访问回调运行在 NimBLE host 任务上，栈空间有限
 * (默认 CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE)。命令层里
 * cmd_status() 需要约 3.2KB 栈 (含 3000 字节的 leds 缓冲)，
 * 直接在该回调里执行会栈溢出。
 *
 * 因此接收回调只做「拼包 + 投递队列」，真正的命令执行放在
 * 独立的 ble_cmd_task (栈 6144) 中，响应再通过 notify 发回。
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_cmd.h"
#include "app_ble.h"

static const char *TAG = "app_ble";

/*
 * 芯片能力检查。
 *
 * ESP32-C6 支持 BLE 5.0；若目标芯片无 BLE (如某些纯 WiFi 型号)
 * 则整个模块退化为空实现。
 */
#if !CONFIG_BT_ENABLED || !SOC_BLE_SUPPORTED

esp_err_t app_ble_start(void)      { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t app_ble_stop(void)       { return ESP_OK; }
bool      app_ble_is_running(void) { return false; }
bool      app_ble_is_connected(void) { return false; }
esp_err_t app_ble_disconnect(void) { return ESP_ERR_INVALID_STATE; }

#else  /* BLE 可用 */

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* ---------------------------------------------------------------------------
 * UUID 定义
 *
 * 用自定义 128 位 UUID，避免与标准服务冲突。
 * 字节序为 BLE 的小端表示 (与 ble_uuid128_t.value 一致)。
 * ------------------------------------------------------------------------- */

/* 服务: 6c6f6f70-0001-4c45-4400-000000000000 */
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44,
    0x45, 0x4c, 0x01, 0x00, 0x70, 0x6f, 0x6f, 0x6c);

/* RX 特征 (写): 6c6f6f70-0002-... */
static const ble_uuid128_t s_rx_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44,
    0x45, 0x4c, 0x02, 0x00, 0x70, 0x6f, 0x6f, 0x6c);

/* TX 特征 (通知): 6c6f6f70-0003-... */
static const ble_uuid128_t s_tx_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44,
    0x45, 0x4c, 0x03, 0x00, 0x70, 0x6f, 0x6f, 0x6c);

/* ---------------------------------------------------------------------------
 * 内部状态
 * ------------------------------------------------------------------------- */

/** 命令接收缓冲区上限 (与 app_cmd 的响应上限对齐) */
#define BLE_RX_BUF_MAX      APP_CMD_RESP_MAX

/** 命令执行任务栈大小 (需容纳 cmd_status 的约 3.2KB) */
#define BLE_CMD_TASK_STACK  6144

/** 命令执行任务优先级 */
#define BLE_CMD_TASK_PRIO   5

/** 命令队列深度 */
#define BLE_CMD_QUEUE_LEN   4

/** 单条待执行命令 */
typedef struct {
    char json[BLE_RX_BUF_MAX];
} ble_cmd_msg_t;

static bool             s_running    = false;
static bool             s_connected  = false;
static uint16_t         s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t          s_own_addr_type = 0;

/** TX 特征值句柄 (notify 时需要) */
static uint16_t         s_tx_val_handle = 0;

/** 命令队列与任务 */
static QueueHandle_t    s_cmd_queue  = NULL;
static TaskHandle_t     s_cmd_task   = NULL;

/* --- 分片接收状态 --- */
static uint8_t          s_rx_buf[BLE_RX_BUF_MAX];
static uint16_t         s_rx_expected = 0;   /*!< 本次命令总长度 (0 = 等待首片) */
static uint16_t         s_rx_got      = 0;   /*!< 已接收字节数 */

/* ---------------------------------------------------------------------------
 * 发送: 把响应按 MTU 分片后逐包 notify
 * ------------------------------------------------------------------------- */

/**
 * @brief 通过 TX 特征发送一段数据 (自动分片)
 *
 * 每片最大长度为 (MTU - 3)，3 字节是 ATT 通知头开销。
 *
 * @param data 数据
 * @param len  长度
 * @return ESP_OK 成功
 */
static esp_err_t ble_send(const char *data, size_t len)
{
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_tx_val_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t mtu = ble_att_mtu(s_conn_handle);
    size_t chunk = (mtu > 3) ? (size_t)(mtu - 3) : 20;

    size_t sent = 0;
    while (sent < len) {
        size_t n = len - sent;
        if (n > chunk) {
            n = chunk;
        }

        struct os_mbuf *om = ble_hs_mbuf_from_flat(data + sent, (uint16_t)n);
        if (om == NULL) {
            ESP_LOGW(TAG, "分配 mbuf 失败");
            return ESP_ERR_NO_MEM;
        }

        int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "notify 失败: rc=%d", rc);
            return ESP_FAIL;
        }

        sent += n;

        /*
         * 分片之间稍作停顿。
         *
         * 连续 notify 会迅速填满控制器的发送缓冲，导致后续调用
         * 返回 BLE_HS_ENOMEM。5 ms 的间隔足以让缓冲排空，
         * 对 3KB 响应 (约 13 片) 只增加 65 ms 延迟，可接受。
         */
        if (sent < len) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * 命令执行任务
 * ------------------------------------------------------------------------- */

static void ble_cmd_task(void *arg)
{
    (void)arg;
    ble_cmd_msg_t *msg = malloc(sizeof(ble_cmd_msg_t));
    if (msg == NULL) {
        ESP_LOGE(TAG, "命令缓冲区分配失败");
        s_cmd_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(s_cmd_queue, msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /*
         * 响应缓冲区用堆分配。
         *
         * 3KB 放在任务栈上会与 cmd_status 内部的 leds[3000] 叠加，
         * 6144 字节的栈不够用。
         */
        char *resp = malloc(APP_CMD_RESP_MAX);
        if (resp == NULL) {
            ESP_LOGE(TAG, "响应缓冲区分配失败");
            continue;
        }

        esp_err_t err = app_cmd_execute(msg->json, resp, APP_CMD_RESP_MAX);
        if (err == ESP_OK) {
            ble_send(resp, strlen(resp));
        } else {
            ESP_LOGW(TAG, "命令执行失败: %s", esp_err_to_name(err));
            ble_send("{\"ok\":false,\"error\":\"内部错误\"}", 33);
        }

        free(resp);
    }
}

/* ---------------------------------------------------------------------------
 * GATT 访问回调
 * ------------------------------------------------------------------------- */

/**
 * @brief RX 特征写入回调
 *
 * 只负责拼包，不执行业务逻辑 (避免占用 NimBLE host 任务栈)。
 *
 * 分片格式:
 *   首片: [总长度 2 字节小端][数据...]
 *   后续: [数据...]
 */
static int gatt_rx_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
    if (om_len == 0) {
        return 0;
    }

    uint8_t  frag[256];
    uint16_t frag_len = 0;

    int rc = ble_hs_mbuf_to_flat(ctxt->om, frag, sizeof(frag), &frag_len);
    if (rc != 0) {
        ESP_LOGW(TAG, "读取写入数据失败: rc=%d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }

    /* --- 首片: 解析总长度 --- */
    if (s_rx_expected == 0) {
        if (frag_len < 2) {
            ESP_LOGW(TAG, "首片过短 (%u 字节)", frag_len);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        s_rx_expected = (uint16_t)(frag[0] | (frag[1] << 8));
        if (s_rx_expected == 0 || s_rx_expected >= BLE_RX_BUF_MAX) {
            ESP_LOGW(TAG, "命令长度非法: %u", s_rx_expected);
            s_rx_expected = 0;
            s_rx_got = 0;
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        s_rx_got = 0;

        uint16_t payload = (uint16_t)(frag_len - 2);
        uint16_t copy = payload;
        if (copy > (uint16_t)(s_rx_expected - s_rx_got)) {
            copy = (uint16_t)(s_rx_expected - s_rx_got);
        }
        memcpy(s_rx_buf, frag + 2, copy);
        s_rx_got = copy;
    } else {
        /* --- 后续片: 直接追加 --- */
        uint16_t copy = frag_len;
        if (copy > (uint16_t)(s_rx_expected - s_rx_got)) {
            copy = (uint16_t)(s_rx_expected - s_rx_got);
        }
        memcpy(s_rx_buf + s_rx_got, frag, copy);
        s_rx_got = (uint16_t)(s_rx_got + copy);
    }

    /* --- 收齐了? --- */
    if (s_rx_got >= s_rx_expected) {
        s_rx_buf[s_rx_expected] = '\0';

        ESP_LOGI(TAG, "收到命令 (%u 字节)", s_rx_expected);

        if (s_cmd_queue != NULL) {
            ble_cmd_msg_t msg;
            memcpy(msg.json, s_rx_buf, (size_t)s_rx_expected + 1);
            if (xQueueSend(s_cmd_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "命令队列已满，丢弃");
            }
        }

        s_rx_expected = 0;
        s_rx_got = 0;
    }

    return 0;
}

/** TX 特征读取回调 (返回能力描述，便于客户端探测) */
static int gatt_tx_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    /* 告知客户端本设备支持的最大命令长度 */
    char info[64];
    snprintf(info, sizeof(info), "{\"max\":%d}", BLE_RX_BUF_MAX - 1);

    int rc = os_mbuf_append(ctxt->om, info, strlen(info));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/** GATT 服务定义 */
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX: 客户端写入命令 */
                .uuid       = &s_rx_uuid.u,
                .access_cb  = gatt_rx_access,
                .flags      = BLE_GATT_CHR_F_WRITE |
                              BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                /* TX: 设备回传响应 (notify) + 可读能力描述 */
                .uuid       = &s_tx_uuid.u,
                .access_cb  = gatt_tx_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_tx_val_handle,
            },
            { 0 }   /* 结束标记 */
        },
    },
    { 0 }   /* 结束标记 */
};

/* ---------------------------------------------------------------------------
 * GAP 事件处理
 * ------------------------------------------------------------------------- */

static int gap_event_handler(struct ble_gap_event *event, void *arg);

/** 开始广播 */
static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    /*
     * 广播内容:
     *   - flags: 通用可发现 + 仅 BLE
     *   - 完整设备名
     *   - 128 位服务 UUID (便于客户端按服务过滤)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)APP_BLE_DEVICE_NAME;
    fields.name_len = strlen(APP_BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    static ble_uuid_any_t svc_uuid_adv;
    ble_uuid_copy(&svc_uuid_adv, &s_svc_uuid.u);
    fields.uuids128 = &svc_uuid_adv.u128;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置广播数据失败: rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   /* 可连接 */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   /* 通用可发现 */

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "启动广播失败: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "BLE 广播中，设备名: %s", APP_BLE_DEVICE_NAME);
    }
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connected   = true;
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "客户端已连接 (handle=%u)", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "连接失败 (status=%d)，重新广播",
                     event->connect.status);
            ble_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "客户端已断开 (reason=%d)", event->disconnect.reason);
        s_connected   = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        /* 复位分片状态，避免残留数据污染下一条命令 */
        s_rx_expected = 0;
        s_rx_got      = 0;
        ble_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGD(TAG, "广播结束，重新开始");
        ble_advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU 已协商: %u (handle=%u)",
                 event->mtu.value, event->mtu.conn_handle);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "客户端订阅: attr=%u notify=%u",
                 event->subscribe.attr_handle,
                 (unsigned)event->subscribe.cur_notify);
        return 0;

    default:
        return 0;
    }
}

/* ---------------------------------------------------------------------------
 * NimBLE 主机回调
 * ------------------------------------------------------------------------- */

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "准备蓝牙地址失败: rc=%d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "推断地址类型失败: rc=%d", rc);
        return;
    }

    ble_advertise();
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE 协议栈复位: reason=%d", reason);
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();              /* 阻塞直到 nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ---------------------------------------------------------------------------
 * 启动 / 停止
 * ------------------------------------------------------------------------- */

esp_err_t app_ble_start(void)
{
    if (s_running) {
        return ESP_ERR_INVALID_STATE;
    }

    /* --- 命令队列与执行任务 --- */
    if (s_cmd_queue == NULL) {
        s_cmd_queue = xQueueCreate(BLE_CMD_QUEUE_LEN, sizeof(ble_cmd_msg_t));
        if (s_cmd_queue == NULL) {
            ESP_LOGE(TAG, "创建命令队列失败");
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_cmd_task == NULL) {
        if (xTaskCreate(ble_cmd_task, "ble_cmd", BLE_CMD_TASK_STACK,
                        NULL, BLE_CMD_TASK_PRIO, &s_cmd_task) != pdPASS) {
            ESP_LOGE(TAG, "创建命令任务失败");
            return ESP_ERR_NO_MEM;
        }
    }

    /* --- NimBLE 协议栈 --- */
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 注册 GAP / GATT 基础服务 */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "统计 GATT 配置失败: rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "注册 GATT 服务失败: rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_svc_gap_device_name_set(APP_BLE_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGW(TAG, "设置设备名失败: rc=%d", rc);
    }

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    /*
     * 不启用配对绑定。
     *
     * 本设备是本地控制用途，绑定会占用 NVS 空间并增加连接复杂度。
     * 若需限制访问，应在应用层做 (例如连接后要求口令)。
     */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;

    nimble_port_freertos_init(ble_host_task);

    s_running = true;
    ESP_LOGI(TAG, "BLE 服务已启动");
    return ESP_OK;
}

esp_err_t app_ble_stop(void)
{
    if (!s_running) {
        return ESP_OK;
    }

    if (s_connected && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    nimble_port_stop();
    nimble_port_deinit();

    s_running     = false;
    s_connected   = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

    ESP_LOGI(TAG, "BLE 服务已停止");
    return ESP_OK;
}

bool app_ble_is_running(void)
{
    return s_running;
}

bool app_ble_is_connected(void)
{
    return s_connected;
}

esp_err_t app_ble_disconnect(void)
{
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
        ESP_LOGW(TAG, "断开连接失败: rc=%d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "已主动断开客户端连接");
    return ESP_OK;
}

#endif  /* BLE 可用 */
