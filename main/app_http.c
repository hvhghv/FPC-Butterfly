/*
 * 蝴蝶板灯控应用 — HTTP 服务模块实现
 *
 * 本文件只是**传输层**: 把 HTTP 请求转成命令层调用，再把响应发回。
 * 所有业务逻辑都在 app_cmd.c 中，与 BLE (app_ble.c) 共用。
 *
 *   HTTP 请求 ──▶ handler ──▶ app_cmd_execute() ──▶ led_ctrl / app_wifi
 *   HTTP 响应 ◀── handler ◀── 响应 JSON        ◀──
 *
 * 路由注册顺序很重要: 精确路径在前，通配符兜底路由必须放在最后。
 * IDF 按注册顺序查找，先命中的精确路由优先。
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_cmd.h"
#include "app_http.h"

static const char *TAG = "app_http";

/** 请求体最大长度 (序列命令较长，放宽) */
#define HTTP_BODY_MAX       512

/** 命令响应缓冲区长度 */
#define HTTP_RESP_MAX       APP_CMD_RESP_MAX

/** HTTP 服务器句柄 */
static httpd_handle_t s_server = NULL;

/* ============================================================================
 * 通用工具
 * ========================================================================== */

/** 发送 JSON 响应 */
static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

/** 发送统一格式的错误响应 */
static esp_err_t send_error(httpd_req_t *req, const char *msg, const char *status)
{
    char body[192];
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", msg);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_status(req, status);
    return httpd_resp_sendstr(req, body);
}

/**
 * @brief 读取请求体到缓冲区
 *
 * @param[in]  req     请求
 * @param[out] buf     输出缓冲区
 * @param[in]  buf_len 缓冲区长度
 * @return 实际读取长度，失败返回 -1
 */
static int read_body(httpd_req_t *req, char *buf, size_t buf_len)
{
    if (req->content_len <= 0) {
        return -1;
    }
    if ((size_t)req->content_len >= buf_len) {
        ESP_LOGW(TAG, "请求体过长: %d 字节 (上限 %u)", req->content_len,
                 (unsigned)(buf_len - 1));
        return -1;
    }

    int received = 0;
    while (received < req->content_len) {
        int ret = httpd_req_recv(req, buf + received,
                                 (size_t)(req->content_len - received));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;   /* 超时重试 */
            }
            return -1;
        }
        received += ret;
    }

    buf[received] = '\0';
    return received;
}

/* ============================================================================
 * Web 控制界面
 *
 * 页面源码位于 main/web/index.html，由 CMake 的 EMBED_FILES 在编译时
 * 自动嵌入为二进制符号 (无需手工转义 C 字符串):
 *
 *   _binary_index_html_start / _binary_index_html_end
 *
 * 符号名由 IDF 根据文件路径生成，路径中的 '/' 与 '.' 均替换为 '_'。
 * 修改页面只需编辑 HTML 文件，重新编译即可生效。
 * ========================================================================== */

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/** 页面字节长度 (编译期嵌入，不含结尾 NUL) */
#define INDEX_HTML_LEN   ((size_t)(index_html_end - index_html_start))

/**
 * @brief 发送 Web 控制界面
 *
 * 嵌入数据不带结尾 NUL，因此必须显式传入长度，不能用 HTTPD_RESP_USE_STRLEN。
 */
static esp_err_t send_index_html(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start,
                           (ssize_t)INDEX_HTML_LEN);
}

/* ============================================================================
 * 命令桥接
 * ========================================================================== */

/**
 * @brief 把请求体交给命令层执行并回传响应
 *
 * 响应缓冲区用堆分配 —— status 命令的响应可达 3KB，
 * 放在 HTTP 任务栈 (8192 字节) 上风险较大。
 *
 * @param req      请求
 * @param cmd_name 命令名 (会注入到 JSON 的 "cmd" 字段)
 * @param body     请求体 JSON，可为 NULL (表示无参数)
 */
static esp_err_t run_cmd(httpd_req_t *req, const char *cmd_name, const char *body)
{
    char *resp = malloc(HTTP_RESP_MAX);
    if (resp == NULL) {
        return send_error(req, "内存不足", "500 Internal Server Error");
    }

    /*
     * 组装最终请求 JSON: 把 cmd 字段注入请求体。
     *
     * 请求体形如 {"id":0,"r":255}，注入后为
     * {"cmd":"led.set","id":0,"r":255}。
     * 这样命令层只需解析一个统一格式的 JSON。
     */
    char *json = malloc(HTTP_BODY_MAX + 64);
    if (json == NULL) {
        free(resp);
        return send_error(req, "内存不足", "500 Internal Server Error");
    }

    if (body != NULL && body[0] == '{') {
        /* 跳过原 JSON 的左花括号，把 cmd 放在最前 */
        snprintf(json, HTTP_BODY_MAX + 64, "{\"cmd\":\"%s\",%s",
                 cmd_name, body + 1);
    } else {
        snprintf(json, HTTP_BODY_MAX + 64, "{\"cmd\":\"%s\"}", cmd_name);
    }

    esp_err_t err = app_cmd_execute_src(json, resp, HTTP_RESP_MAX,
                                        APP_CMD_SRC_HTTP);

    free(json);

    if (err == ESP_ERR_INVALID_SIZE) {
        free(resp);
        return send_error(req, "响应过长", "500 Internal Server Error");
    }
    if (err != ESP_OK) {
        free(resp);
        return send_error(req, esp_err_to_name(err), "500 Internal Server Error");
    }

    esp_err_t ret = send_json(req, resp);
    free(resp);
    return ret;
}

/** 读取请求体并执行命令 (无请求体时按空参数处理) */
static esp_err_t run_cmd_with_body(httpd_req_t *req, const char *cmd_name)
{
    char body[HTTP_BODY_MAX];

    if (req->content_len <= 0) {
        /* 无请求体，例如 POST /api/off */
        return run_cmd(req, cmd_name, NULL);
    }

    if (read_body(req, body, sizeof(body)) < 0) {
        return send_error(req, "请求体无效", "400 Bad Request");
    }
    return run_cmd(req, cmd_name, body);
}

/* ============================================================================
 * 路由处理函数
 * ========================================================================== */

/** GET / — Web 控制界面 */
static esp_err_t handler_index(httpd_req_t *req)
{
    return send_index_html(req);
}

/** GET /api/status — 灯珠状态与系统信息 */
static esp_err_t handler_status(httpd_req_t *req)
{
    return run_cmd(req, "status", NULL);
}

/** POST /api/led — 设置单颗灯珠完整状态 */
static esp_err_t handler_led(httpd_req_t *req)
{
    return run_cmd_with_body(req, "led.set");
}

/** POST /api/led/effect — 设置单颗灯珠效果 */
static esp_err_t handler_led_effect(httpd_req_t *req)
{
    return run_cmd_with_body(req, "led.effect");
}

/** POST /api/led/brightness — 设置单颗灯珠亮度 */
static esp_err_t handler_led_brightness(httpd_req_t *req)
{
    return run_cmd_with_body(req, "led.brightness");
}

/** POST /api/led/enable — 启用/禁用单颗灯珠 */
static esp_err_t handler_led_enable(httpd_req_t *req)
{
    return run_cmd_with_body(req, "led.enable");
}

/** POST /api/led/sequence — 设置单颗灯珠效果序列 */
static esp_err_t handler_led_sequence(httpd_req_t *req)
{
    return run_cmd_with_body(req, "led.sequence");
}

/** POST /api/all — 设置全部灯珠颜色 */
static esp_err_t handler_all(httpd_req_t *req)
{
    return run_cmd_with_body(req, "all");
}

/** POST /api/brightness — 设置全局亮度 */
static esp_err_t handler_brightness(httpd_req_t *req)
{
    return run_cmd_with_body(req, "brightness");
}

/** POST /api/effect — 设置全局效果 */
static esp_err_t handler_effect(httpd_req_t *req)
{
    return run_cmd_with_body(req, "effect");
}

/** POST /api/off — 全部熄灭 */
static esp_err_t handler_off(httpd_req_t *req)
{
    return run_cmd_with_body(req, "off");
}

/** POST /api/freq — 设置 PWM 频率 */
static esp_err_t handler_freq(httpd_req_t *req)
{
    return run_cmd_with_body(req, "freq");
}

/** POST /api/reboot — 重启设备 */
static esp_err_t handler_reboot(httpd_req_t *req)
{
    ESP_LOGW(TAG, "收到重启请求");
    return run_cmd(req, "reboot", NULL);
}

/** POST /api/upgrade — 重启进入 IAP 下载模式 */
static esp_err_t handler_upgrade(httpd_req_t *req)
{
    ESP_LOGW(TAG, "收到升级请求，准备进入 IAP 下载模式");
    return run_cmd(req, "upgrade", NULL);
}

/** GET /api/wifi — 读取 WiFi 配置与 STA 状态 */
static esp_err_t handler_wifi_get(httpd_req_t *req)
{
    return run_cmd(req, "wifi.get", NULL);
}

/** POST /api/wifi — 修改 WiFi 配置 */
static esp_err_t handler_wifi_set(httpd_req_t *req)
{
    return run_cmd_with_body(req, "wifi.set");
}

/** POST /api/wifi/scan — 扫描周边热点 */
static esp_err_t handler_wifi_scan(httpd_req_t *req)
{
    return run_cmd(req, "wifi.scan", NULL);
}

/** POST /api/wifi/reconnect — 手动触发 STA 重连 */
static esp_err_t handler_wifi_reconnect(httpd_req_t *req)
{
    return run_cmd(req, "wifi.reconnect", NULL);
}

/** POST /api/wifi/reset — 恢复默认 WiFi 配置并重启 */
static esp_err_t handler_wifi_reset(httpd_req_t *req)
{
    return run_cmd(req, "wifi.reset", NULL);
}

/** POST /api/wifi/enable — 启用/禁用 WiFi (需 confirm:1) */
static esp_err_t handler_wifi_enable(httpd_req_t *req)
{
    return run_cmd_with_body(req, "wifi.enable");
}

/** POST /api/ble/enable — 启用/禁用蓝牙 (需 confirm:1) */
static esp_err_t handler_ble_enable(httpd_req_t *req)
{
    return run_cmd_with_body(req, "ble.enable");
}

/** POST /api/ble/pin — 查询/设置蓝牙配对码 */
static esp_err_t handler_ble_pin(httpd_req_t *req)
{
    return run_cmd_with_body(req, "ble.pin");
}

/** GET /api/config — 导出配置 (不含密码) */
static esp_err_t handler_config_export(httpd_req_t *req)
{
    return run_cmd(req, "config.export", NULL);
}

/**
 * POST /api/config/export — 导出配置
 *
 * 请求体 {"secrets":1} 时包含密码明文，用于完整迁移。
 * 默认不含密码。
 */
static esp_err_t handler_config_export_post(httpd_req_t *req)
{
    return run_cmd_with_body(req, "config.export");
}

/** POST /api/config — 导入配置 */
static esp_err_t handler_config_import(httpd_req_t *req)
{
    return run_cmd_with_body(req, "config.import");
}

/** 兜底路由 (通配符路径) — 返回主页面 */
static esp_err_t handler_catch_all(httpd_req_t *req)
{
    ESP_LOGD(TAG, "未匹配路径 %s，返回主页面", req->uri);
    return send_index_html(req);
}


static const httpd_uri_t s_uris[] = {
    { .uri = "/",              .method = HTTP_GET,  .handler = handler_index },
    { .uri = "/api/status",    .method = HTTP_GET,  .handler = handler_status },
    { .uri = "/api/led",       .method = HTTP_POST, .handler = handler_led },
    { .uri = "/api/led/effect",.method = HTTP_POST, .handler = handler_led_effect },
    { .uri = "/api/led/brightness", .method = HTTP_POST, .handler = handler_led_brightness },
    { .uri = "/api/led/enable",.method = HTTP_POST, .handler = handler_led_enable },
    { .uri = "/api/led/sequence", .method = HTTP_POST, .handler = handler_led_sequence },
    { .uri = "/api/all",       .method = HTTP_POST, .handler = handler_all },
    { .uri = "/api/brightness",.method = HTTP_POST, .handler = handler_brightness },
    { .uri = "/api/effect",    .method = HTTP_POST, .handler = handler_effect },
    { .uri = "/api/off",       .method = HTTP_POST, .handler = handler_off },
    { .uri = "/api/freq",      .method = HTTP_POST, .handler = handler_freq },
    { .uri = "/api/reboot",    .method = HTTP_POST, .handler = handler_reboot },
    { .uri = "/api/upgrade",   .method = HTTP_POST, .handler = handler_upgrade },
    { .uri = "/api/wifi",      .method = HTTP_GET,  .handler = handler_wifi_get },
    { .uri = "/api/wifi",      .method = HTTP_POST, .handler = handler_wifi_set },
    { .uri = "/api/wifi/scan", .method = HTTP_POST, .handler = handler_wifi_scan },
    { .uri = "/api/wifi/reconnect", .method = HTTP_POST, .handler = handler_wifi_reconnect },
    { .uri = "/api/wifi/reset",.method = HTTP_POST, .handler = handler_wifi_reset },
    { .uri = "/api/wifi/enable", .method = HTTP_POST, .handler = handler_wifi_enable },
    { .uri = "/api/ble/enable", .method = HTTP_POST, .handler = handler_ble_enable },
    { .uri = "/api/ble/pin",   .method = HTTP_POST, .handler = handler_ble_pin },
    { .uri = "/api/config",    .method = HTTP_GET,  .handler = handler_config_export },
    { .uri = "/api/config",    .method = HTTP_POST, .handler = handler_config_import },
    { .uri = "/api/config/export", .method = HTTP_POST, .handler = handler_config_export_post },
    /* 兜底路由必须放在最后 */
    { .uri = "/*",             .method = HTTP_GET,  .handler = handler_catch_all },
};

/* ============================================================================
 * 启动 / 停止
 * ========================================================================== */

esp_err_t app_http_start(void)
{
    if (s_server != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers  = sizeof(s_uris) / sizeof(s_uris[0]) + 2;
    config.stack_size        = 8192;
    config.lru_purge_enable  = true;
    /*
     * 超时设置:
     *   recv_wait_timeout: 等待请求头的秒数
     *   send_wait_timeout: 发送响应的秒数
     *
     * 浏览器会保持长连接 (keep-alive)，若超时过短，连接被服务端关闭后
     * 浏览器可能复用已关闭的 socket，导致:
     *   httpd_sock_err: error in send : 11  (EAGAIN)
     *   httpd_sock_err: error in send : 104 (ECONNRESET)
     * 因此适当放宽。
     *
     * 注意: status 响应约 3KB，在 WiFi 拥塞时单次发送可能耗时较久，
     * send_wait_timeout 太小会中途失败 (408 Request Timeout)。
     */
    config.recv_wait_timeout = 20;
    config.send_wait_timeout = 30;
    /*
     * 并发连接数:
     *   浏览器通常同时开 2-6 个连接 (页面 + favicon + 轮询 XHR)。
     *   默认 4 个偏少，容易触发 LRU 清理导致连接被强制关闭。
     *   注意: 每连接约占用 1-2KB，需与空闲堆平衡。
     */
    config.max_open_sockets  = 7;
    /* 使用通配符匹配以支持兜底路由 */
    config.uri_match_fn      = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动 HTTP 服务失败: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    for (size_t i = 0; i < sizeof(s_uris) / sizeof(s_uris[0]); i++) {
        err = httpd_register_uri_handler(s_server, &s_uris[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "注册路由 %s 失败: %s",
                     s_uris[i].uri, esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "HTTP 服务已启动");
    return ESP_OK;
}

esp_err_t app_http_stop(void)
{
    if (s_server == NULL) {
        return ESP_OK;
    }

    esp_err_t err = httpd_stop(s_server);
    s_server = NULL;

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP 服务已停止");
    } else {
        ESP_LOGW(TAG, "停止 HTTP 服务失败: %s", esp_err_to_name(err));
    }
    return err;
}

bool app_http_is_running(void)
{
    return s_server != NULL;
}
