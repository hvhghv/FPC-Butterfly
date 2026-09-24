/*
 * 蝴蝶板灯珠 HTTP 控制应用 — 主入口
 *
 * 本程序是 ESP IAP 用户程序，运行在 user_app 分区 (0x150000)，
 * 由自定义 bootloader 根据 RTC RAM 中的启动决策加载。
 *
 * 【v5 架构要点】
 *
 *   1. 独立分区表
 *      本程序使用**分区表 B** (位于 0x140000)，与 IAP 的分区表 A
 *      (0xB000) 完全独立。本程序**看不到** IAP 的分区。
 *
 *   2. iap_cfg 不可访问
 *      配置区 (0x8000) 位于 IDF flash 写保护区 (0x0 ~ 0xBFFF) 内，
 *      且是 IAP 私有数据。本程序**不得**读写，只能通过启动参数只读获取。
 *
 *   3. RTC RAM 传递启动参数
 *      IAP 启动本程序前把 iap_boot_param_t 写入 RTC RAM
 *      (rtc_retain_mem_t.custom[])，本程序启动后读取。
 *
 *   4. 独立 NVS
 *      分区表 B 中的 NVS 分区名为 "nvs" (0x141000)，与 IAP 的 "nvs"
 *      (0xC000) 物理地址不同，互不干扰。
 *      注意: 必须叫 "nvs" —— IDF 的 nvs_open() 硬编码该名字，
 *            WiFi 等组件依赖它。
 *
 * 启动流程:
 *   1. 读取 RTC RAM 启动参数（mode / reason / boot / baud）
 *   2. 初始化灯珠 PWM (12 路: LEDC 6 + MCPWM 6)
 *   3. 启动 WiFi SoftAP
 *   4. 启动 HTTP 服务，提供 Web 控制界面与 REST API
 *
 * 硬件 (4 颗共阴极 RGB 灯珠):
 *
 *   | 灯珠 | 位置坐标   | 红    | 绿    | 蓝    | 公共阴极 |
 *   |------|------------|-------|-------|-------|----------|
 *   | D1   | (17, 22)mm | IO4   | IO5   | IO6   | GND      |
 *   | D2   | (22, 39)mm | IO0   | IO1   | IO7   | GND      |
 *   | D3   | (53, 22)mm | IO18  | IO19  | IO20  | GND      |
 *   | D4   | (48, 39)mm | IO21  | IO22  | IO23  | GND      |
 *
 * 编译: idf.py set-target esp32c6 && idf.py build
 * 烧录: esptool write_flash 0x140000 build/led_butterfly_flash.bin
 *       (合并镜像: 分区表 B @ 0x140000 + 应用镜像 @ 0x150000)
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "iap_user_api.h"
#include "iap_boot_param.h"
#include "led_ctrl.h"
#include "app_wifi.h"
#include "app_http.h"
#include "app_ble.h"

static const char *TAG = "led_app";

/**
 * 本程序版本号 (0xMMmmpp 格式)
 *
 * v5: 版本号不再上报到配置区，仅用于本地日志与 HTTP 显示。
 */
#define APP_VERSION   0x010000    /* 1.0.0 */

/* ============================================================================
 * 应用配置（从 RTC RAM 启动参数读取）
 * ========================================================================== */

typedef struct {
    char     mode[16];          /*!< 运行模式: normal / debug / factory */
    char     reason[24];        /*!< 进入 IAP 的原因 */
    uint32_t boot_count;        /*!< 启动计数 */
    uint32_t baudrate;          /*!< 建议通信波特率 */
} app_config_t;

static app_config_t s_app_cfg = {
    .mode       = "normal",
    .reason     = "unknown",
    .boot_count = 0,
    .baudrate   = 115200,
};

/* 升级请求队列（示例：任何任务都可投递） */
static QueueHandle_t s_upgrade_req = NULL;

/* ============================================================================
 * 启动参数解析
 * ========================================================================== */

/**
 * @brief 从 IAP 启动参数中加载应用配置
 *
 * 启动参数格式: "mode=normal;reason=wait_timeout;boot=3;baud=921600"
 */
static void load_boot_params(void)
{
    /*
     * 用 static 避免占用栈空间。
     * iap_user_get_boot_param() 内部会再分配 4096 字节槽缓冲，
     * 叠加本缓冲后极易溢出 main 任务栈。
     * 本函数仅在启动时调用一次，无重入风险。
     */
    static char param[IAP_USER_BOOT_PARAM_SIZE];

    esp_err_t err = iap_user_get_boot_param(param, sizeof(param));
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "无启动参数，使用默认配置");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "读取启动参数失败: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "启动参数: %s", param);

    char val[32];

    if (iap_user_boot_param_get_value(param, "mode", val, sizeof(val)) == ESP_OK) {
        strlcpy(s_app_cfg.mode, val, sizeof(s_app_cfg.mode));
    }
    if (iap_user_boot_param_get_value(param, "reason", val, sizeof(val)) == ESP_OK) {
        strlcpy(s_app_cfg.reason, val, sizeof(s_app_cfg.reason));
    }
    if (iap_user_boot_param_get_value(param, "boot", val, sizeof(val)) == ESP_OK) {
        s_app_cfg.boot_count = (uint32_t)strtoul(val, NULL, 10);
    }
    if (iap_user_boot_param_get_value(param, "baud", val, sizeof(val)) == ESP_OK) {
        uint32_t b = (uint32_t)strtoul(val, NULL, 10);
        if (b >= 9600) {
            s_app_cfg.baudrate = b;
        }
    }

    ESP_LOGI(TAG, "模式=%s 原因=%s 启动计数=%" PRIu32 " 波特率=%" PRIu32,
             s_app_cfg.mode, s_app_cfg.reason,
             s_app_cfg.boot_count, s_app_cfg.baudrate);
}

/**
 * @brief 按运行模式初始化
 *
 * IAP 下载完成后可以指定一次性的运行模式（如 factory 自检、debug 详细日志），
 * 用户程序据此分支。
 */
static void init_by_mode(void)
{
    if (strcmp(s_app_cfg.mode, "debug") == 0) {
        esp_log_level_set("*", ESP_LOG_DEBUG);
        ESP_LOGI(TAG, "进入调试模式（详细日志）");
    } else if (strcmp(s_app_cfg.mode, "factory") == 0) {
        ESP_LOGI(TAG, "进入出厂自检模式");
    } else {
        ESP_LOGI(TAG, "进入正常工作模式");
    }
}

/* ============================================================================
 * 读取 RTC RAM 启动参数
 *
 * v5: 配置区 (0x8000) 对用户程序**不开放**（位于 IDF flash 写保护区内，
 *     且是 IAP 私有数据）。本程序只能通过 RTC RAM 只读获取启动信息。
 * ========================================================================== */

static void dump_boot_param(void)
{
    /* 用 static 避免占用栈空间 */
    static iap_user_boot_param_t bp;

    if (iap_user_boot_param_read(&bp) != ESP_OK) {
        ESP_LOGW(TAG, "读取 RTC RAM 启动参数失败");
        return;
    }

    ESP_LOGI(TAG, "RTC RAM 启动参数: v%u, target=%u",
             (unsigned)bp.version, (unsigned)bp.boot_target);
    ESP_LOGI(TAG, "  APP 地址=0x%08" PRIX32 " 大小=%" PRIu32 " 版本=0x%06" PRIX32,
             bp.user_addr, bp.user_size, bp.user_version);
    ESP_LOGI(TAG, "  iap_cfg 地址=0x%08" PRIX32 " (本程序不可访问)",
             bp.cfg_addr);
}

/* ============================================================================
 * 升级请求
 * ========================================================================== */

/**
 * @brief 请求升级（线程安全，可从任意任务/中断投递）
 *
 * 实际重启在 upgrade_task 中执行，避免在中断上下文调用。
 */
void app_request_upgrade(void)
{
    if (s_upgrade_req != NULL) {
        uint8_t dummy = 1;
        xQueueSend(s_upgrade_req, &dummy, pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 升级请求处理任务
 *
 * 收到请求后置位下载标志并重启，IAP 启动时会进入下载模式。
 */
static void upgrade_task(void *arg)
{
    (void)arg;
    uint8_t dummy;

    while (1) {
        if (xQueueReceive(s_upgrade_req, &dummy, portMAX_DELAY) == pdTRUE) {
            ESP_LOGW(TAG, "收到升级请求，%d 秒后重启进入 IAP 下载模式...", 2);
            vTaskDelay(pdMS_TO_TICKS(2000));

            /* 置位下载标志并重启 — 正常情况不会返回 */
            esp_err_t err = iap_user_request_download();
            ESP_LOGE(TAG, "请求下载失败: %s", esp_err_to_name(err));
        }
    }
}

/* ============================================================================
 * 上电自检动画
 * ========================================================================== */

/**
 * @brief 依次点亮 4 颗灯珠（红 -> 绿 -> 蓝）
 *
 * 用于直观确认 GPIO 映射与灯珠顺序是否正确。
 * 每颗灯珠停留 250 ms，最后全白闪一下再熄灭。
 */
static void boot_animation(void)
{
    const uint8_t colors[3][3] = {
        { 255, 0,   0   },  /* 红 */
        { 0,   255, 0   },  /* 绿 */
        { 0,   0,   255 },  /* 蓝 */
    };

    ESP_LOGI(TAG, "执行上电自检动画");

    for (int c = 0; c < 3; c++) {
        for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
            led_ctrl_set_all(0, 0, 0);
            led_ctrl_set_rgb(i, colors[c][0], colors[c][1], colors[c][2]);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

    led_ctrl_set_all(255, 255, 255);
    vTaskDelay(pdMS_TO_TICKS(400));
    led_ctrl_set_all(0, 0, 0);
    led_ctrl_all_off();
}

/* ============================================================================
 * 业务逻辑
 * ========================================================================== */

/**
 * @brief 运行状态汇报任务
 *
 * 每 10 秒打印一次运行状态，便于通过串口观察设备健康状况。
 */
static void status_task(void *arg)
{
    (void)arg;
    int64_t start = esp_timer_get_time() / 1000000;
    int tick = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        tick++;

        int64_t uptime = esp_timer_get_time() / 1000000 - start;
        led_effect_t effect = LED_EFFECT_NONE;
        led_ctrl_get_effect(&effect, NULL);

        ESP_LOGI(TAG, "[%d] 运行 %lld 秒 | 空闲堆 %" PRIu32
                 " | 效果=%s | 亮度=%u | 客户端=%u | http://%s/",
                 tick, uptime, (uint32_t)esp_get_free_heap_size(),
                 led_effect_names[effect], led_ctrl_get_brightness(),
                 app_wifi_get_sta_count(), app_wifi_get_ip());
    }
}

/* ============================================================================
 * 入口
 * ========================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  蝴蝶板灯珠 HTTP 控制应用 v1.0.0");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "本程序版本: 0x%06" PRIX32, (uint32_t)APP_VERSION);

    /* --- 1. 读取 RTC RAM 启动参数 ---------------------------------------- */
    load_boot_params();
    init_by_mode();

    /* --- 2. 打印 RTC RAM 启动信息 ---------------------------------------- */
    dump_boot_param();

    /* --- 3. 初始化 NVS --------------------------------------------------- */
    /*
     * v5: 分区表 B 中的 NVS 分区名为 "nvs"（不是 "nvs_app"）。
     *
     * ⚠️ 为什么必须叫 "nvs" 而不是自定义名:
     *    IDF 的 nvs_open() 硬编码使用 NVS_DEFAULT_PART_NAME = "nvs"
     *    (nvs.h:57，不可配置)。WiFi / BT 等组件内部都通过 nvs_open()
     *    访问 NVS，因此分区必须命名为 "nvs"，否则:
     *      esp_wifi_init() -> nvs_open_wrapper() -> nvs_open()
     *      -> 找不到 "nvs" 分区 -> ESP_ERR_NVS_PART_NOT_FOUND
     *
     *    用 nvs_flash_init_partition("nvs") 与 nvs_flash_init() 等价，
     *    这里显式写分区名以便与 IAP 侧的 nvs_iap 对照。
     *
     *    隔离性: 分区表 B 的 "nvs" 在 0x141000，IAP 的 "nvs" 在 0xC000，
     *            物理地址不同，互不干扰。
     */
    esp_err_t nvs_err = nvs_flash_init_partition("nvs");
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase_partition("nvs");
        nvs_err = nvs_flash_init_partition("nvs");
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGW(TAG, "NVS 初始化失败: %s", esp_err_to_name(nvs_err));
    }

    /* --- 4. 初始化灯珠 PWM (12 路: LEDC 6 + MCPWM 6) --------------------- */
    esp_err_t err = led_ctrl_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "灯珠初始化失败: %s", esp_err_to_name(err));
        /* 灯珠不可用时仍继续启动网络服务，便于远程诊断 */
    } else {
        boot_animation();
    }

    /* --- 5. 启动 WiFi (AP / STA / APSTA，按 NVS 配置) -------------------- */
    err = app_wifi_start();
    if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "本芯片不支持 WiFi，HTTP 服务不可用");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 启动失败: %s", esp_err_to_name(err));
    } else {
        /* --- 6. 启动 HTTP 服务 ------------------------------------------- */
        err = app_http_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "HTTP 服务启动失败: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "----------------------------------------");
            ESP_LOGI(TAG, "  控制页面: http://%s/", app_wifi_get_ip());
            ESP_LOGI(TAG, "  AP SSID : %s", app_wifi_get_ssid());
            ESP_LOGI(TAG, "----------------------------------------");
        }
    }

    /* --- 6b. 启动 BLE 蓝牙控制通道 --------------------------------------- */
    /*
     * BLE 与 HTTP 是**并列**的两条控制通道，共用 app_cmd 命令层。
     * BLE 启动失败不影响 WiFi/HTTP 通道，因此只告警不中断。
     */
    err = app_ble_start();
    if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "本芯片不支持 BLE，蓝牙控制不可用");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE 启动失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "  蓝牙名称: %s", APP_BLE_DEVICE_NAME);
    }

    /* --- 7. 创建升级请求队列与任务 --------------------------------------- */
    s_upgrade_req = xQueueCreate(4, sizeof(uint8_t));
    if (s_upgrade_req == NULL) {
        ESP_LOGE(TAG, "创建升级队列失败");
    } else {
        xTaskCreate(upgrade_task, "upgrade", 3072, NULL, 5, NULL);
    }

    /* --- 8. 启动状态汇报任务 -------------------------------------------- */
    xTaskCreate(status_task, "status", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG, "初始化完成，等待 HTTP 控制请求");
}
