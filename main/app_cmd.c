/*
 * 蝴蝶板灯控应用 — 命令层实现
 *
 * 见 app_cmd.h 的架构说明。
 *
 * 本文件承载所有控制逻辑，HTTP 与 BLE 只负责传输。
 * 命令名与 HTTP 路径一一对应，便于对照排查:
 *
 *   HTTP 路径              命令名
 *   GET  /api/status    -> "status"
 *   POST /api/led       -> "led.set"
 *   POST /api/led/effect     -> "led.effect"
 *   POST /api/led/brightness -> "led.brightness"
 *   POST /api/led/enable     -> "led.enable"
 *   POST /api/led/sequence   -> "led.sequence"
 *   POST /api/all       -> "all"
 *   POST /api/brightness-> "brightness"
 *   POST /api/effect    -> "effect"
 *   POST /api/off       -> "off"
 *   POST /api/freq      -> "freq"
 *   GET  /api/wifi      -> "wifi.get"
 *   POST /api/wifi      -> "wifi.set"
 *   POST /api/wifi/scan -> "wifi.scan"
 *   POST /api/wifi/reconnect -> "wifi.reconnect"
 *   POST /api/reboot    -> "reboot"
 *   POST /api/upgrade   -> "upgrade"
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "esp_app_desc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "iap_user_api.h"
#include "iap_boot_param.h"
#include "app_wifi.h"
#include "app_ble.h"
#include "led_ctrl.h"
#include "app_cmd.h"

static const char *TAG = "app_cmd";

/* ============================================================================
 * JSON 工具
 * ========================================================================== */

bool app_cmd_get_int(const char *json, const char *key, long *out)
{
    if (json == NULL || key == NULL || out == NULL) {
        return false;
    }

    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *p = json;
    while ((p = strstr(p, pattern)) != NULL) {
        p += strlen(pattern);

        /* 跳过空白与冒号 */
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
            p++;
        }
        if (*p != ':') {
            continue;   /* 不是键值对，继续向后找 */
        }
        p++;

        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
            p++;
        }

        char *end = NULL;
        long v = strtol(p, &end, 10);
        if (end == p) {
            return false;   /* 不是数字 */
        }

        *out = v;
        return true;
    }

    return false;
}

bool app_cmd_get_str(const char *json, const char *key,
                     char *out, size_t out_len)
{
    if (json == NULL || key == NULL || out == NULL || out_len == 0) {
        return false;
    }

    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *p = strstr(json, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);

    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    if (*p != ':') {
        return false;
    }
    p++;

    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    if (*p != '"') {
        return false;
    }
    p++;

    const char *end = strchr(p, '"');
    if (end == NULL) {
        return false;
    }

    size_t len = (size_t)(end - p);
    if (len >= out_len) {
        len = out_len - 1;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

long app_cmd_clamp(long v, long lo, long hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** 写入统一格式的错误响应 */
static esp_err_t reply_error(char *buf, size_t len, const char *msg)
{
    int n = snprintf(buf, len, "{\"ok\":false,\"error\":\"%s\"}", msg);
    return (n < 0 || (size_t)n >= len) ? ESP_ERR_INVALID_SIZE : ESP_OK;
}

/** 写入统一格式的成功响应 (无附加数据) */
static esp_err_t reply_ok(char *buf, size_t len)
{
    int n = snprintf(buf, len, "{\"ok\":true}");
    return (n < 0 || (size_t)n >= len) ? ESP_ERR_INVALID_SIZE : ESP_OK;
}

/* ============================================================================
 * 命令: status — 灯珠状态 + 系统信息
 * ========================================================================== */

static esp_err_t cmd_status(char *out, size_t out_len)
{
    /* --- 灯珠状态 (含逐颗参数与序列) --- */
    char leds[3000];
    esp_err_t err = led_ctrl_to_json(leds, sizeof(leds));
    if (err != ESP_OK) {
        return reply_error(out, out_len, "读取灯珠状态失败");
    }

    /* --- 系统信息 --- */
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    const char *model;
    switch (chip.model) {
    case CHIP_ESP32:    model = "ESP32";    break;
    case CHIP_ESP32S2:  model = "ESP32-S2"; break;
    case CHIP_ESP32S3:  model = "ESP32-S3"; break;
    case CHIP_ESP32C3:  model = "ESP32-C3"; break;
    case CHIP_ESP32C6:  model = "ESP32-C6"; break;
    case CHIP_ESP32H2:  model = "ESP32-H2"; break;
    default:            model = "未知";      break;
    }

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    const esp_app_desc_t *desc = esp_app_get_description();

    /*
     * 从 RTC RAM 启动参数获取 IAP 传递的元数据。
     *
     * 用户程序**不能**访问 iap_cfg 配置区 (位于 flash 写保护区内)，
     * 因此启动计数等信息只能来自 RTC RAM。
     */
    static iap_user_boot_param_t bp;
    uint32_t user_version = 0;
    uint32_t cfg_addr = 0;
    uint8_t  boot_target = 0xFF;

    if (iap_user_boot_param_read(&bp) == ESP_OK) {
        user_version = bp.user_version;
        cfg_addr     = bp.cfg_addr;
        boot_target  = bp.boot_target;
    }

    char iap_ver_str[16];
    if (user_version != 0) {
        snprintf(iap_ver_str, sizeof(iap_ver_str),
                 "%" PRIu32 ".%" PRIu32 ".%" PRIu32,
                 (user_version >> 16) & 0xFF,
                 (user_version >> 8) & 0xFF,
                 user_version & 0xFF);
    } else {
        snprintf(iap_ver_str, sizeof(iap_ver_str), "未提供");
    }

    /* --- 组装 JSON ---
     * leds 已含 count/leds/brightness/effect/period/freq 字段，
     * 这里去掉最外层花括号后并入总响应。
     */
    const char *leds_body = leds;
    size_t leds_len = strlen(leds);
    if (leds_len >= 2 && leds[0] == '{' && leds[leds_len - 1] == '}') {
        leds_body = leds + 1;
        leds_len -= 2;
    }

    /*
     * 固定字段部分的最大长度估算:
     *   chip(16) + cores(4) + idf(16) + app(48) + mac(24) + heap(12)
     *   + uptime(16) + boottarget(4) + iapver(16) + cfgaddr(12)
     *   + ssid(40) + ip(20) + clients(4)
     *   + ble 对象(约 80) + wifi 对象(约 260)
     *   + 键名与标点(约 200)
     *   ≈ 800 字节，取 1024 留足余量。
     */
    const size_t fixed_max = 1024;
    size_t need = leds_len + fixed_max + 2;
    if (need > out_len) {
        return reply_error(out, out_len, "响应过长");
    }

    /* --- WiFi 状态 --- */
    app_wifi_sta_status_t st;
    if (app_wifi_get_sta_status(&st) != ESP_OK) {
        memset(&st, 0, sizeof(st));
    }

    int n = snprintf(out, out_len,
                     "{\"chip\":\"%s\",\"cores\":%d,\"idf\":\"%s\","
                     "\"app\":\"%s %s\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                     "\"heap\":%" PRIu32 ",\"uptime\":%" PRIu64 ","
                     "\"boottarget\":%u,\"iapver\":\"%s\","
                     "\"cfgaddr\":%" PRIu32 ","
                     "\"ssid\":\"%s\",\"ip\":\"%s\",\"clients\":%u,"
                     "\"ble\":{\"running\":%s,\"connected\":%s,\"name\":\"%s\"},"
                     "\"wifi\":{\"mode\":%u,\"ap_clients\":%u,"
                     "\"sta_enabled\":%s,\"sta_connected\":%s,"
                     "\"sta_static\":%s,\"sta_ssid\":\"%s\","
                     "\"sta_ip\":\"%s\",\"sta_gw\":\"%s\",\"sta_mask\":\"%s\","
                     "\"sta_dns\":\"%s\",\"sta_rssi\":%d,\"sta_channel\":%u},",
                     model, chip.cores, esp_get_idf_version(),
                     desc->project_name, desc->version,
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     (uint32_t)esp_get_free_heap_size(),
                     (uint64_t)(esp_timer_get_time() / 1000000),
                     (unsigned)boot_target, iap_ver_str, cfg_addr,
                     app_wifi_get_ssid(), app_wifi_get_ip(),
                     app_wifi_get_sta_count(),
                     app_ble_is_running() ? "true" : "false",
                     app_ble_is_connected() ? "true" : "false",
                     APP_BLE_DEVICE_NAME,
                     app_wifi_get_mode(),
                     app_wifi_get_sta_count(),
                     st.enabled ? "true" : "false",
                     st.connected ? "true" : "false",
                     st.static_ip ? "true" : "false",
                     st.ssid, st.ip, st.gw, st.mask, st.dns,
                     (int)st.rssi, st.channel);

    /* 必须检查 snprintf 返回值，否则截断时会越界写 */
    if (n < 0 || (size_t)n + leds_len + 2 > out_len) {
        return reply_error(out, out_len, "响应过长");
    }

    memcpy(out + n, leds_body, leds_len);
    n += (int)leds_len;
    out[n++] = '}';
    out[n] = '\0';

    return ESP_OK;
}

/* ============================================================================
 * 命令: led.* — 逐颗灯珠控制
 * ========================================================================== */

/** 解析 id 字段，返回 -1 表示非法 */
static long parse_id(const char *json, char *err, size_t err_len)
{
    long id = 0;
    if (!app_cmd_get_int(json, "id", &id)) {
        snprintf(err, err_len, "缺少 id 字段");
        return -1;
    }
    if (id < 0 || id >= LED_CTRL_COUNT) {
        snprintf(err, err_len, "id 超出范围 (0-3)");
        return -1;
    }
    return id;
}

static esp_err_t cmd_led_set(const char *json, char *out, size_t out_len)
{
    char err[64];
    long id = parse_id(json, err, sizeof(err));
    if (id < 0) {
        return reply_error(out, out_len, err);
    }

    /* --- 颜色: 三个通道要么都提供，要么都不提供 --- */
    long r = 0, g = 0, b = 0;
    bool has_rgb = app_cmd_get_int(json, "r", &r) |
                   app_cmd_get_int(json, "g", &g) |
                   app_cmd_get_int(json, "b", &b);

    uint8_t rv = 0, gv = 0, bv = 0;
    if (has_rgb) {
        rv = (uint8_t)app_cmd_clamp(r, 0, 255);
        gv = (uint8_t)app_cmd_clamp(g, 0, 255);
        bv = (uint8_t)app_cmd_clamp(b, 0, 255);
    }

    /* --- 亮度 --- */
    long bri = 0;
    bool has_bri = app_cmd_get_int(json, "brightness", &bri);
    uint8_t briv = (uint8_t)app_cmd_clamp(bri, 0, 255);

    /* --- 效果 --- */
    char name[24];
    bool has_eff = app_cmd_get_str(json, "effect", name, sizeof(name));
    led_effect_t eff = LED_EFFECT_NONE;
    if (has_eff) {
        eff = led_ctrl_effect_from_name(name);
        if (eff >= LED_EFFECT_MAX) {
            return reply_error(out, out_len, "未知效果名称");
        }
    }

    /* --- 效果周期 --- */
    long period = 0;
    bool has_period = app_cmd_get_int(json, "period", &period);
    uint32_t periodv = (uint32_t)app_cmd_clamp(period, 100, 60000);

    /* --- 开关 --- */
    long en = 0;
    bool has_en = app_cmd_get_int(json, "enabled", &en);
    bool env = (en != 0);

    esp_err_t e = led_ctrl_set_led_state((uint8_t)id,
                                         has_rgb ? &rv : NULL,
                                         has_rgb ? &gv : NULL,
                                         has_rgb ? &bv : NULL,
                                         has_bri ? &briv : NULL,
                                         has_eff ? &eff : NULL,
                                         has_period ? &periodv : NULL,
                                         has_en ? &env : NULL);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len,
             "{\"ok\":true,\"id\":%ld,\"r\":%u,\"g\":%u,\"b\":%u,"
             "\"brightness\":%u,\"effect\":\"%s\",\"period\":%" PRIu32 ","
             "\"enabled\":%s}",
             id, rv, gv, bv, briv,
             (eff < LED_EFFECT_MAX) ? led_effect_names[eff] : "none",
             periodv, env ? "true" : "false");
    return ESP_OK;
}

static esp_err_t cmd_led_effect(const char *json, char *out, size_t out_len)
{
    char err[64];
    long id = parse_id(json, err, sizeof(err));
    if (id < 0) {
        return reply_error(out, out_len, err);
    }

    char name[24];
    if (!app_cmd_get_str(json, "name", name, sizeof(name))) {
        return reply_error(out, out_len, "缺少 name 字段");
    }

    led_effect_t effect = led_ctrl_effect_from_name(name);
    if (effect >= LED_EFFECT_MAX) {
        return reply_error(out, out_len, "未知效果名称");
    }

    long period = 0;
    app_cmd_get_int(json, "period", &period);
    if (period != 0) {
        period = app_cmd_clamp(period, 100, 60000);
    }

    esp_err_t e = led_ctrl_set_led_effect((uint8_t)id, effect, (uint32_t)period);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len,
             "{\"ok\":true,\"id\":%ld,\"effect\":\"%s\",\"period\":%ld}",
             id, name, period);
    return ESP_OK;
}

static esp_err_t cmd_led_brightness(const char *json, char *out, size_t out_len)
{
    char err[64];
    long id = parse_id(json, err, sizeof(err));
    if (id < 0) {
        return reply_error(out, out_len, err);
    }

    long v = 0;
    if (!app_cmd_get_int(json, "value", &v)) {
        return reply_error(out, out_len, "缺少 value 字段");
    }
    v = app_cmd_clamp(v, 0, 255);

    esp_err_t e = led_ctrl_set_led_brightness((uint8_t)id, (uint8_t)v);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len, "{\"ok\":true,\"id\":%ld,\"brightness\":%ld}", id, v);
    return ESP_OK;
}

static esp_err_t cmd_led_enable(const char *json, char *out, size_t out_len)
{
    char err[64];
    long id = parse_id(json, err, sizeof(err));
    if (id < 0) {
        return reply_error(out, out_len, err);
    }

    long en = 0;
    if (!app_cmd_get_int(json, "enabled", &en)) {
        return reply_error(out, out_len, "缺少 enabled 字段");
    }

    esp_err_t e = led_ctrl_set_led_enabled((uint8_t)id, en != 0);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len, "{\"ok\":true,\"id\":%ld,\"enabled\":%s}",
             id, (en != 0) ? "true" : "false");
    return ESP_OK;
}

static esp_err_t cmd_led_sequence(const char *json, char *out, size_t out_len)
{
    char err[64];
    long id = parse_id(json, err, sizeof(err));
    if (id < 0) {
        return reply_error(out, out_len, err);
    }

    /* --- 定位 steps 数组 --- */
    const char *p = strstr(json, "\"steps\"");
    if (p == NULL) {
        return reply_error(out, out_len, "缺少 steps 字段");
    }
    p = strchr(p, '[');
    if (p == NULL) {
        return reply_error(out, out_len, "steps 不是数组");
    }
    p++;

    /*
     * 逐对象解析: 找到每个 '{' ... '}'，在其中提取 effect/duration/period。
     * 极简解析器，不处理嵌套对象或数组。
     */
    led_step_t steps[LED_SEQ_MAX_STEPS];
    uint8_t    count = 0;

    while (*p != '\0' && *p != ']') {
        if (*p != '{') {
            p++;
            continue;
        }

        const char *obj_end = strchr(p, '}');
        if (obj_end == NULL) {
            break;      /* JSON 不完整 */
        }

        size_t obj_len = (size_t)(obj_end - p) + 1;
        char obj[160];
        if (obj_len >= sizeof(obj)) {
            return reply_error(out, out_len, "步骤过长");
        }
        memcpy(obj, p, obj_len);
        obj[obj_len] = '\0';

        if (count >= LED_SEQ_MAX_STEPS) {
            return reply_error(out, out_len, "步骤数超过上限 (8)");
        }

        /* effect (必填) */
        char name[24];
        if (!app_cmd_get_str(obj, "effect", name, sizeof(name))) {
            return reply_error(out, out_len, "步骤缺少 effect 字段");
        }
        led_effect_t eff = led_ctrl_effect_from_name(name);
        if (eff >= LED_EFFECT_MAX) {
            return reply_error(out, out_len, "步骤中有未知效果名称");
        }

        /* duration (必填) */
        long dur = 0;
        if (!app_cmd_get_int(obj, "duration", &dur)) {
            return reply_error(out, out_len, "步骤缺少 duration 字段");
        }
        dur = app_cmd_clamp(dur, 10, 600000);

        /* period (可选，0 = 与 duration 相同) */
        long per = 0;
        app_cmd_get_int(obj, "period", &per);
        if (per != 0) {
            per = app_cmd_clamp(per, 100, 60000);
        }

        steps[count].effect      = (uint8_t)eff;
        steps[count].duration_ms = (uint32_t)dur;
        steps[count].period_ms   = (uint32_t)per;
        count++;

        p = obj_end + 1;
    }

    esp_err_t e = led_ctrl_set_led_sequence((uint8_t)id, steps, count);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len, "{\"ok\":true,\"id\":%ld,\"steps\":%u}",
             id, (unsigned)count);
    return ESP_OK;
}

/* ============================================================================
 * 命令: 全局灯控
 * ========================================================================== */

static esp_err_t cmd_all(const char *json, char *out, size_t out_len)
{
    long r = 0, g = 0, b = 0;
    app_cmd_get_int(json, "r", &r);
    app_cmd_get_int(json, "g", &g);
    app_cmd_get_int(json, "b", &b);

    r = app_cmd_clamp(r, 0, 255);
    g = app_cmd_clamp(g, 0, 255);
    b = app_cmd_clamp(b, 0, 255);

    esp_err_t e = led_ctrl_set_all((uint8_t)r, (uint8_t)g, (uint8_t)b);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len, "{\"ok\":true,\"r\":%ld,\"g\":%ld,\"b\":%ld}", r, g, b);
    return ESP_OK;
}

static esp_err_t cmd_brightness(const char *json, char *out, size_t out_len)
{
    long v = 0;
    if (!app_cmd_get_int(json, "value", &v)) {
        return reply_error(out, out_len, "缺少 value 字段");
    }
    v = app_cmd_clamp(v, 0, 255);

    esp_err_t e = led_ctrl_set_brightness((uint8_t)v);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len, "{\"ok\":true,\"brightness\":%ld}", v);
    return ESP_OK;
}

static esp_err_t cmd_effect(const char *json, char *out, size_t out_len)
{
    char name[24];
    if (!app_cmd_get_str(json, "name", name, sizeof(name))) {
        return reply_error(out, out_len, "缺少 name 字段");
    }

    led_effect_t effect = led_ctrl_effect_from_name(name);
    if (effect >= LED_EFFECT_MAX) {
        return reply_error(out, out_len, "未知效果名称");
    }

    long period = 0;
    app_cmd_get_int(json, "period", &period);
    if (period != 0) {
        period = app_cmd_clamp(period, 100, 60000);
    }

    esp_err_t e = led_ctrl_set_effect(effect, (uint32_t)period);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len, "{\"ok\":true,\"effect\":\"%s\",\"period\":%ld}",
             name, period);
    return ESP_OK;
}

static esp_err_t cmd_off(char *out, size_t out_len)
{
    /* 关闭效果，否则呼吸/彩虹会立刻重新点亮 */
    led_ctrl_set_effect(LED_EFFECT_NONE, 0);

    esp_err_t e = led_ctrl_all_off();
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }
    return reply_ok(out, out_len);
}

static esp_err_t cmd_freq(const char *json, char *out, size_t out_len)
{
    long freq = 0;
    if (!app_cmd_get_int(json, "freq", &freq)) {
        return reply_error(out, out_len, "缺少 freq 字段");
    }
    freq = app_cmd_clamp(freq, 100, 40000);

    esp_err_t e = led_ctrl_set_freq((uint32_t)freq);
    if (e != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(e));
    }

    snprintf(out, out_len, "{\"ok\":true,\"freq\":%ld}", freq);
    return ESP_OK;
}

/* ============================================================================
 * 命令: wifi.* — 网络配置
 * ========================================================================== */

static esp_err_t cmd_wifi_get(char *out, size_t out_len)
{
    app_wifi_cfg_t cfg;
    esp_err_t err = app_wifi_get_cfg(&cfg);
    if (err != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(err));
    }

    app_wifi_sta_status_t st;
    if (app_wifi_get_sta_status(&st) != ESP_OK) {
        memset(&st, 0, sizeof(st));
    }

    size_t ap_pass_len  = strlen(cfg.password);
    size_t sta_pass_len = strlen(cfg.sta_password);

    snprintf(out, out_len,
             "{\"ok\":true,\"mode\":%u,"
             "\"ssid\":\"%s\",\"channel\":%u,\"maxconn\":%u,"
             "\"hidden\":%s,\"ip\":\"%s\",\"secure\":%s,\"passlen\":%u,"
             "\"sta\":{\"enabled\":%s,\"connected\":%s,\"static\":%s,"
             "\"ssid\":\"%s\",\"ip\":\"%s\",\"gw\":\"%s\",\"mask\":\"%s\","
             "\"dns\":\"%s\",\"rssi\":%d,\"channel\":%u,"
             "\"bssid\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
             "\"secure\":%s,\"passlen\":%u,"
             "\"cfg_ip\":\"%s\",\"cfg_mask\":\"%s\",\"cfg_gw\":\"%s\","
             "\"cfg_dns\":\"%s\"}}",
             cfg.mode,
             cfg.ssid, cfg.channel, cfg.max_conn,
             cfg.hidden ? "true" : "false", cfg.ip,
             (ap_pass_len >= 8) ? "true" : "false", (unsigned)ap_pass_len,
             st.enabled ? "true" : "false",
             st.connected ? "true" : "false",
             st.static_ip ? "true" : "false",
             st.ssid, st.ip, st.gw, st.mask, st.dns,
             (int)st.rssi, st.channel,
             st.bssid[0], st.bssid[1], st.bssid[2],
             st.bssid[3], st.bssid[4], st.bssid[5],
             (sta_pass_len >= 8) ? "true" : "false", (unsigned)sta_pass_len,
             cfg.sta_ip, cfg.sta_mask, cfg.sta_gw, cfg.sta_dns);
    return ESP_OK;
}

static esp_err_t cmd_wifi_set(const char *json, char *out, size_t out_len)
{
    /* 以当前配置为基准，只覆盖请求中出现的字段 */
    app_wifi_cfg_t cfg;
    esp_err_t err = app_wifi_get_cfg(&cfg);
    if (err != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(err));
    }

    long v = 0;
    if (app_cmd_get_int(json, "mode", &v)) {
        cfg.mode = (uint8_t)app_cmd_clamp(v, 0, 2);
    }

    char ssid[APP_WIFI_SSID_MAX];
    if (app_cmd_get_str(json, "ssid", ssid, sizeof(ssid))) {
        snprintf(cfg.ssid, sizeof(cfg.ssid), "%s", ssid);
    }

    /*
     * 密码区分「未提供」与「显式设为空」:
     *   未提供 -> 保持原密码；提供 "" -> 开放网络。
     * app_cmd_get_str() 对空串返回 true，正好满足需求。
     */
    char pass[APP_WIFI_PASS_MAX];
    if (app_cmd_get_str(json, "password", pass, sizeof(pass))) {
        snprintf(cfg.password, sizeof(cfg.password), "%s", pass);
    }

    if (app_cmd_get_int(json, "channel", &v)) {
        cfg.channel = (uint8_t)app_cmd_clamp(v, 1, 13);
    }
    if (app_cmd_get_int(json, "maxconn", &v)) {
        cfg.max_conn = (uint8_t)app_cmd_clamp(v, 1, 10);
    }
    if (app_cmd_get_int(json, "hidden", &v)) {
        cfg.hidden = (v != 0);
    }

    char ip[APP_WIFI_IP_MAX];
    if (app_cmd_get_str(json, "ip", ip, sizeof(ip))) {
        snprintf(cfg.ip, sizeof(cfg.ip), "%s", ip);
    }

    char sta_ssid[APP_WIFI_SSID_MAX];
    if (app_cmd_get_str(json, "sta_ssid", sta_ssid, sizeof(sta_ssid))) {
        snprintf(cfg.sta_ssid, sizeof(cfg.sta_ssid), "%s", sta_ssid);
    }

    char sta_pass[APP_WIFI_PASS_MAX];
    if (app_cmd_get_str(json, "sta_password", sta_pass, sizeof(sta_pass))) {
        snprintf(cfg.sta_password, sizeof(cfg.sta_password), "%s", sta_pass);
    }

    /* --- STA 静态 IP --- */
    if (app_cmd_get_int(json, "sta_static", &v)) {
        cfg.sta_static_ip = (v != 0);
    }
    char sip[APP_WIFI_IP_MAX];
    if (app_cmd_get_str(json, "sta_ip", sip, sizeof(sip))) {
        snprintf(cfg.sta_ip, sizeof(cfg.sta_ip), "%s", sip);
    }
    char smask[APP_WIFI_IP_MAX];
    if (app_cmd_get_str(json, "sta_mask", smask, sizeof(smask))) {
        snprintf(cfg.sta_mask, sizeof(cfg.sta_mask), "%s", smask);
    }
    char sgw[APP_WIFI_IP_MAX];
    if (app_cmd_get_str(json, "sta_gw", sgw, sizeof(sgw))) {
        snprintf(cfg.sta_gw, sizeof(cfg.sta_gw), "%s", sgw);
    }
    char sdns[APP_WIFI_IP_MAX];
    if (app_cmd_get_str(json, "sta_dns", sdns, sizeof(sdns))) {
        snprintf(cfg.sta_dns, sizeof(cfg.sta_dns), "%s", sdns);
    }

    err = app_wifi_set_cfg(&cfg);
    if (err == ESP_ERR_INVALID_ARG) {
        return reply_error(out, out_len,
                           "参数非法 (SSID 1-32 / 密码 8-63 或空 / "
                           "信道 1-13 / 连接数 1-10 / IP 需以 .1 结尾 / "
                           "静态 IP 需与网关同网段且不能相同 / "
                           "模式含 STA 时需填路由器 SSID)");
    }
    if (err != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(err));
    }

    snprintf(out, out_len,
             "{\"ok\":true,\"mode\":%u,\"ssid\":\"%s\",\"channel\":%u,"
             "\"maxconn\":%u,\"hidden\":%s,\"ip\":\"%s\","
             "\"sta_ssid\":\"%s\",\"sta_static\":%s,"
             "\"sta_ip\":\"%s\",\"sta_mask\":\"%s\",\"sta_gw\":\"%s\","
             "\"sta_dns\":\"%s\"}",
             cfg.mode, cfg.ssid, cfg.channel, cfg.max_conn,
             cfg.hidden ? "true" : "false", cfg.ip, cfg.sta_ssid,
             cfg.sta_static_ip ? "true" : "false",
             cfg.sta_ip, cfg.sta_mask, cfg.sta_gw, cfg.sta_dns);
    return ESP_OK;
}

static esp_err_t cmd_wifi_scan(char *out, size_t out_len)
{
    app_wifi_ap_info_t list[APP_WIFI_SCAN_MAX];
    uint8_t found = 0;

    esp_err_t err = app_wifi_scan(list, APP_WIFI_SCAN_MAX, &found);
    if (err == ESP_ERR_INVALID_STATE) {
        return reply_error(out, out_len, "WiFi 未启动");
    }
    if (err != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(err));
    }

    int n = snprintf(out, out_len, "{\"ok\":true,\"count\":%u,\"aps\":[",
                     (unsigned)found);

    for (uint8_t i = 0; i < found; i++) {
        n += snprintf(out + n, out_len - (size_t)n,
                      "%s{\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%u,"
                      "\"secure\":%s}",
                      i ? "," : "", list[i].ssid, (int)list[i].rssi,
                      list[i].channel,
                      (list[i].authmode == 0) ? "false" : "true");
        if (n >= (int)out_len) {
            return reply_error(out, out_len, "扫描结果过长");
        }
    }

    snprintf(out + n, out_len - (size_t)n, "]}");
    return ESP_OK;
}

static esp_err_t cmd_wifi_reconnect(char *out, size_t out_len)
{
    esp_err_t err = app_wifi_sta_reconnect();
    if (err == ESP_ERR_INVALID_STATE) {
        return reply_error(out, out_len, "STA 未启用或未配置路由器");
    }
    if (err != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(err));
    }
    snprintf(out, out_len, "{\"ok\":true,\"reconnecting\":true}");
    return ESP_OK;
}

/* ============================================================================
 * 命令: 服务开关 (wifi.enable / ble.enable)
 *
 * 关闭服务是有风险的操作 —— 尤其关闭 WiFi 会**立即断开当前连接**，
 * 若设备没有其他接入方式 (AP 关闭且 STA 未连上) 将彻底失联。
 * 因此:
 *   - 需要显式传 confirm:1，否则拒绝执行
 *   - 关闭 WiFi 前检查是否有退路 (AP 关闭时 STA 必须已连接)
 * ========================================================================== */

/** 检查参数中是否带 confirm:1 */
static bool has_confirm(const char *json)
{
    long v = 0;
    return app_cmd_get_int(json, "confirm", &v) && v == 1;
}

static esp_err_t cmd_ble_enable(const char *json, char *out, size_t out_len)
{
    long en = 0;
    if (!app_cmd_get_int(json, "enabled", &en)) {
        return reply_error(out, out_len, "缺少 enabled 字段");
    }
    if (!has_confirm(json)) {
        return reply_error(out, out_len,
                           "关闭/启用蓝牙需二次确认 (confirm:1)");
    }

    bool want = (en != 0);

    if (want) {
        esp_err_t err = app_ble_start();
        if (err == ESP_ERR_INVALID_STATE) {
            snprintf(out, out_len, "{\"ok\":true,\"enabled\":true,"
                                   "\"note\":\"已在运行\"}");
            return ESP_OK;
        }
        if (err == ESP_ERR_NOT_SUPPORTED) {
            return reply_error(out, out_len, "本芯片不支持蓝牙");
        }
        if (err != ESP_OK) {
            return reply_error(out, out_len, esp_err_to_name(err));
        }
    } else {
        esp_err_t err = app_ble_stop();
        if (err != ESP_OK) {
            return reply_error(out, out_len, esp_err_to_name(err));
        }
    }

    snprintf(out, out_len, "{\"ok\":true,\"enabled\":%s}",
             want ? "true" : "false");
    return ESP_OK;
}

static esp_err_t cmd_wifi_enable(const char *json, char *out, size_t out_len)
{
    long en = 0;
    if (!app_cmd_get_int(json, "enabled", &en)) {
        return reply_error(out, out_len, "缺少 enabled 字段");
    }
    if (!has_confirm(json)) {
        return reply_error(out, out_len,
                           "关闭/启用 WiFi 需二次确认 (confirm:1)");
    }

    bool want = (en != 0);

    if (want) {
        esp_err_t err = app_wifi_start();
        if (err == ESP_ERR_NOT_SUPPORTED) {
            return reply_error(out, out_len, "本芯片不支持 WiFi");
        }
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return reply_error(out, out_len, esp_err_to_name(err));
        }
    } else {
        /*
         * 关闭前检查退路。
         *
         * 若当前模式含 STA 且已连上路由器，关闭 WiFi 后仍可通过
         * 路由器访问 —— 但那是同一个 WiFi 服务，关掉就都没了。
         * 因此关闭 WiFi 意味着**必然失联**，这里只做告警提示。
         */
        esp_err_t err = app_wifi_stop();
        if (err != ESP_OK) {
            return reply_error(out, out_len, esp_err_to_name(err));
        }
        snprintf(out, out_len,
                 "{\"ok\":true,\"enabled\":false,"
                 "\"warning\":\"WiFi 已关闭，设备将无法通过网络访问，"
                 "只能通过蓝牙或串口恢复\"}");
        return ESP_OK;
    }

    snprintf(out, out_len, "{\"ok\":true,\"enabled\":%s}",
             want ? "true" : "false");
    return ESP_OK;
}

/* ============================================================================
 * 命令: BLE 配对码 (ble.pin)
 * ========================================================================== */

static esp_err_t cmd_ble_pin(const char *json, char *out, size_t out_len)
{
    /* --- 查询 --- */
    long get = 0;
    if (app_cmd_get_int(json, "get", &get) && get == 1) {
        snprintf(out, out_len,
                 "{\"ok\":true,\"enabled\":%s,\"pin\":\"%s\"}",
                 app_ble_pairing_enabled() ? "true" : "false",
                 app_ble_get_pin());
        return ESP_OK;
    }

    /* --- 设置 --- */
    char pin[16];
    const char *p = NULL;
    if (app_cmd_get_str(json, "pin", pin, sizeof(pin))) {
        p = pin;
    }
    /*
     * 允许显式清空: {"pin":""} 或 {"enabled":0}
     * 前者由 get_str 返回空串，后者用 enabled 字段表达。
     */
    long en = 0;
    if (app_cmd_get_int(json, "enabled", &en) && en == 0) {
        p = "";
    }

    if (p == NULL) {
        return reply_error(out, out_len,
                           "缺少 pin 字段 (6 位数字，或空串以禁用配对)");
    }

    esp_err_t err = app_ble_set_pin(p);
    if (err == ESP_ERR_INVALID_ARG) {
        return reply_error(out, out_len, "配对码必须是 6 位数字");
    }
    if (err != ESP_OK) {
        return reply_error(out, out_len, esp_err_to_name(err));
    }

    snprintf(out, out_len,
             "{\"ok\":true,\"enabled\":%s,\"pin\":\"%s\"}",
             app_ble_pairing_enabled() ? "true" : "false",
             app_ble_get_pin());
    return ESP_OK;
}

/* ============================================================================
 * 配置导出 / 导入
 *
 * 导出为一份自包含的 JSON，包含:
 *   - 全局: PWM 频率、输出反转
 *   - 每颗灯珠: 颜色、亮度、效果、周期、开关、效果序列
 *   - WiFi: 模式、AP 参数、STA 参数
 *
 * 出于安全考虑**不导出密码明文**，只导出 secure/passlen 标记。
 * 导入时密码字段可选 —— 不提供则保留设备当前密码。
 * ========================================================================== */

/** 配置格式版本 (将来结构变更时用于兼容处理) */
#define CFG_VERSION  1

/**
 * @brief 导出配置
 *
 * 响应可能超过 3KB (4 颗灯珠 × 8 步序列)，因此调用方需提供
 * 足够大的缓冲区。见 APP_CMD_RESP_MAX。
 *
 * @param json    请求 JSON，可含 "secrets":1 以导出密码明文
 * @param out     输出缓冲区
 * @param out_len 缓冲区长度
 */
static esp_err_t cmd_config_export(const char *json, char *out, size_t out_len)
{
    /*
     * 是否导出密码明文。
     *
     * 默认关闭 —— 导出的文件可能被分享或存云端，明文密码风险高。
     * 仅在用户显式请求 (完整迁移场景) 时才包含。
     */
    long secrets = 0;
    app_cmd_get_int(json, "secrets", &secrets);
    bool with_secrets = (secrets != 0);

    /* --- 全局 LED 参数 --- */
    uint32_t freq   = led_ctrl_get_freq();
    bool     invert = led_ctrl_get_invert();

    int n = snprintf(out, out_len,
                     "{\"ok\":true,\"v\":%d,\"freq\":%" PRIu32 ","
                     "\"invert\":%s,\"secrets\":%s,\"leds\":[",
                     CFG_VERSION, freq, invert ? "true" : "false",
                     with_secrets ? "true" : "false");

    /* --- 逐颗灯珠 --- */
    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        uint8_t  r = 0, g = 0, b = 0;
        led_ctrl_get_rgb(i, &r, &g, &b);

        uint8_t  bri = led_ctrl_get_led_brightness(i);
        bool     en  = led_ctrl_get_led_enabled(i);

        led_effect_t eff = LED_EFFECT_NONE;
        uint32_t period = 0;
        led_ctrl_get_led_effect(i, &eff, &period);

        n += snprintf(out + n, out_len - (size_t)n,
                      "%s{\"id\":%u,\"r\":%u,\"g\":%u,\"b\":%u,"
                      "\"brightness\":%u,\"effect\":\"%s\",\"period\":%" PRIu32 ","
                      "\"enabled\":%s,\"steps\":[",
                      i ? "," : "", (unsigned)(i + 1),
                      r, g, b, bri,
                      (eff < LED_EFFECT_MAX) ? led_effect_names[eff] : "none",
                      period, en ? "true" : "false");

        if (n >= (int)out_len) {
            return reply_error(out, out_len, "配置过长");
        }

        /* 效果序列 */
        led_step_t steps[LED_SEQ_MAX_STEPS];
        uint8_t    cnt = LED_SEQ_MAX_STEPS;
        if (led_ctrl_get_led_sequence(i, steps, &cnt) != ESP_OK) {
            cnt = 0;
        }

        for (uint8_t k = 0; k < cnt; k++) {
            n += snprintf(out + n, out_len - (size_t)n,
                          "%s{\"effect\":\"%s\",\"duration\":%" PRIu32 ","
                          "\"period\":%" PRIu32 "}",
                          k ? "," : "",
                          (steps[k].effect < LED_EFFECT_MAX)
                              ? led_effect_names[steps[k].effect] : "none",
                          steps[k].duration_ms, steps[k].period_ms);
            if (n >= (int)out_len) {
                return reply_error(out, out_len, "配置过长");
            }
        }

        n += snprintf(out + n, out_len - (size_t)n, "]}");
        if (n >= (int)out_len) {
            return reply_error(out, out_len, "配置过长");
        }
    }

    /* --- WiFi 配置 --- */
    app_wifi_cfg_t wcfg;
    if (app_wifi_get_cfg(&wcfg) != ESP_OK) {
        memset(&wcfg, 0, sizeof(wcfg));
    }

    n += snprintf(out + n, out_len - (size_t)n,
                  "],\"wifi\":{\"mode\":%u,\"ssid\":\"%s\",\"channel\":%u,"
                  "\"maxconn\":%u,\"hidden\":%s,\"ip\":\"%s\","
                  "\"secure\":%s,\"passlen\":%u,"
                  "\"sta_ssid\":\"%s\",\"sta_secure\":%s,\"sta_passlen\":%u",
                  wcfg.mode, wcfg.ssid, wcfg.channel, wcfg.max_conn,
                  wcfg.hidden ? "true" : "false", wcfg.ip,
                  (strlen(wcfg.password) >= 8) ? "true" : "false",
                  (unsigned)strlen(wcfg.password),
                  wcfg.sta_ssid,
                  (strlen(wcfg.sta_password) >= 8) ? "true" : "false",
                  (unsigned)strlen(wcfg.sta_password));

    if (n >= (int)out_len) {
        return reply_error(out, out_len, "配置过长");
    }

    /*
     * 仅在显式请求时导出密码明文。
     *
     * 用 password / sta_password 字段名，与 wifi.set 命令一致，
     * 因此导出的 wifi 对象可直接喂给 wifi.set 复用。
     */
    if (with_secrets) {
        n += snprintf(out + n, out_len - (size_t)n,
                      ",\"password\":\"%s\",\"sta_password\":\"%s\"",
                      wcfg.password, wcfg.sta_password);
        if (n >= (int)out_len) {
            return reply_error(out, out_len, "配置过长");
        }
    }

    n += snprintf(out + n, out_len - (size_t)n, "}}");
    if (n >= (int)out_len) {
        return reply_error(out, out_len, "配置过长");
    }

    return ESP_OK;
}

/**
 * @brief 从 JSON 中提取一个灯珠对象并应用
 *
 * @param obj     该灯珠的 JSON 对象文本
 * @param idx     灯珠索引 (0-3)
 * @param applied 输出: 成功应用的灯珠数 (累加)
 * @return ESP_OK 成功
 */
static esp_err_t apply_led_from_json(const char *obj, uint8_t idx)
{
    /* --- 颜色与亮度 --- */
    long r = 0, g = 0, b = 0, bri = 0;
    bool has_rgb = app_cmd_get_int(obj, "r", &r) |
                   app_cmd_get_int(obj, "g", &g) |
                   app_cmd_get_int(obj, "b", &b);
    bool has_bri = app_cmd_get_int(obj, "brightness", &bri);

    uint8_t rv = (uint8_t)app_cmd_clamp(r, 0, 255);
    uint8_t gv = (uint8_t)app_cmd_clamp(g, 0, 255);
    uint8_t bv = (uint8_t)app_cmd_clamp(b, 0, 255);
    uint8_t briv = (uint8_t)app_cmd_clamp(bri, 0, 255);

    /* --- 效果 --- */
    char name[24];
    bool has_eff = app_cmd_get_str(obj, "effect", name, sizeof(name));
    led_effect_t eff = LED_EFFECT_NONE;
    if (has_eff) {
        eff = led_ctrl_effect_from_name(name);
        if (eff >= LED_EFFECT_MAX) {
            eff = LED_EFFECT_NONE;      /* 未知效果降级为静态 */
        }
    }

    long period = 0;
    bool has_period = app_cmd_get_int(obj, "period", &period);
    uint32_t periodv = (uint32_t)app_cmd_clamp(period, 100, 60000);

    long en = 0;
    bool has_en = app_cmd_get_int(obj, "enabled", &en);
    bool env = (en != 0);

    led_ctrl_set_led_state(idx,
                           has_rgb ? &rv : NULL,
                           has_rgb ? &gv : NULL,
                           has_rgb ? &bv : NULL,
                           has_bri ? &briv : NULL,
                           has_eff ? &eff : NULL,
                           has_period ? &periodv : NULL,
                           has_en ? &env : NULL);

    /* --- 效果序列 --- */
    const char *sp = strstr(obj, "\"steps\"");
    if (sp != NULL) {
        sp = strchr(sp, '[');
        if (sp != NULL) {
            sp++;

            led_step_t steps[LED_SEQ_MAX_STEPS];
            uint8_t    count = 0;

            while (*sp != '\0' && *sp != ']') {
                if (*sp != '{') {
                    sp++;
                    continue;
                }
                const char *end = strchr(sp, '}');
                if (end == NULL) {
                    break;
                }

                size_t olen = (size_t)(end - sp) + 1;
                char sobj[160];
                if (olen >= sizeof(sobj)) {
                    break;      /* 步骤过长，跳过 */
                }
                memcpy(sobj, sp, olen);
                sobj[olen] = '\0';

                if (count >= LED_SEQ_MAX_STEPS) {
                    break;      /* 超出上限，忽略多余步骤 */
                }

                char sname[24];
                led_effect_t seff = LED_EFFECT_NONE;
                if (app_cmd_get_str(sobj, "effect", sname, sizeof(sname))) {
                    seff = led_ctrl_effect_from_name(sname);
                    if (seff >= LED_EFFECT_MAX) {
                        seff = LED_EFFECT_NONE;
                    }
                }

                long dur = 1000, per = 0;
                app_cmd_get_int(sobj, "duration", &dur);
                app_cmd_get_int(sobj, "period", &per);

                steps[count].effect      = (uint8_t)seff;
                steps[count].duration_ms = (uint32_t)app_cmd_clamp(dur, 10, 600000);
                steps[count].period_ms   = (uint32_t)((per != 0)
                                            ? app_cmd_clamp(per, 100, 60000) : 0);
                count++;

                sp = end + 1;
            }

            led_ctrl_set_led_sequence(idx, steps, count);
        }
    }

    return ESP_OK;
}

/**
 * @brief 导入配置
 *
 * 请求体为 config.export 输出的 JSON (可原样回传)。
 * WiFi 密码字段可选: 未提供则保留设备当前密码。
 */
static esp_err_t cmd_config_import(const char *json, char *out, size_t out_len)
{
    /* --- 版本检查 --- */
    long ver = 0;
    if (app_cmd_get_int(json, "v", &ver)) {
        if (ver > CFG_VERSION) {
            return reply_error(out, out_len,
                               "配置版本过新，请升级固件后再导入");
        }
    }

    uint8_t led_applied = 0;

    /* --- 逐颗灯珠 --- */
    const char *p = strstr(json, "\"leds\"");
    if (p != NULL) {
        p = strchr(p, '[');
        if (p != NULL) {
            p++;
            uint8_t idx = 0;

            while (*p != '\0' && *p != ']' && idx < LED_CTRL_COUNT) {
                if (*p != '{') {
                    p++;
                    continue;
                }

                /*
                 * 找到匹配的 '}'。
                 *
                 * 灯珠对象内含嵌套的 "steps":[...]，但 steps 里是
                 * 对象数组而非对象套对象，因此第一个 '}' 就是灯珠对象的
                 * 结束 —— 除非 steps 为空数组，那也同样是第一个 '}'。
                 * 这里用括号计数更稳妥。
                 */
                const char *q = p;
                int depth = 0;
                const char *end = NULL;
                while (*q != '\0') {
                    if (*q == '{') depth++;
                    else if (*q == '}') {
                        depth--;
                        if (depth == 0) { end = q; break; }
                    }
                    q++;
                }
                if (end == NULL) {
                    break;
                }

                size_t olen = (size_t)(end - p) + 1;
                if (olen >= 2048) {
                    break;      /* 对象过大，异常 */
                }

                char *obj = malloc(olen + 1);
                if (obj == NULL) {
                    return reply_error(out, out_len, "内存不足");
                }
                memcpy(obj, p, olen);
                obj[olen] = '\0';

                apply_led_from_json(obj, idx);
                led_applied++;
                idx++;

                free(obj);
                p = end + 1;
            }
        }
    }

    /* --- 全局 LED 参数 --- */
    long freq = 0;
    if (app_cmd_get_int(json, "freq", &freq)) {
        freq = app_cmd_clamp(freq, 100, 40000);
        led_ctrl_set_freq((uint32_t)freq);
    }

    long inv = 0;
    if (app_cmd_get_int(json, "invert", &inv)) {
        led_ctrl_set_invert(inv != 0);
    }

    /* --- WiFi 配置 --- */
    bool wifi_applied = false;
    bool pass_restored = false;     /*!< 是否恢复了密码 (完整迁移) */
    const char *wp = strstr(json, "\"wifi\"");
    if (wp != NULL) {
        const char *wb = strchr(wp, '{');
        if (wb != NULL) {
            /* 取到 wifi 对象结束 */
            const char *q = wb;
            int depth = 0;
            const char *wend = NULL;
            while (*q != '\0') {
                if (*q == '{') depth++;
                else if (*q == '}') {
                    depth--;
                    if (depth == 0) { wend = q; break; }
                }
                q++;
            }

            if (wend != NULL) {
                size_t wlen = (size_t)(wend - wb) + 1;
                char *wobj = malloc(wlen + 1);
                if (wobj != NULL) {
                    memcpy(wobj, wb, wlen);
                    wobj[wlen] = '\0';

                    app_wifi_cfg_t wcfg;
                    if (app_wifi_get_cfg(&wcfg) == ESP_OK) {
                        long v = 0;
                        if (app_cmd_get_int(wobj, "mode", &v)) {
                            wcfg.mode = (uint8_t)app_cmd_clamp(v, 0, 2);
                        }
                        char s[APP_WIFI_SSID_MAX];
                        if (app_cmd_get_str(wobj, "ssid", s, sizeof(s))) {
                            snprintf(wcfg.ssid, sizeof(wcfg.ssid), "%s", s);
                        }
                        if (app_cmd_get_int(wobj, "channel", &v)) {
                            wcfg.channel = (uint8_t)app_cmd_clamp(v, 1, 13);
                        }
                        if (app_cmd_get_int(wobj, "maxconn", &v)) {
                            wcfg.max_conn = (uint8_t)app_cmd_clamp(v, 1, 10);
                        }
                        if (app_cmd_get_int(wobj, "hidden", &v)) {
                            wcfg.hidden = (v != 0);
                        }
                        char ip[APP_WIFI_IP_MAX];
                        if (app_cmd_get_str(wobj, "ip", ip, sizeof(ip))) {
                            snprintf(wcfg.ip, sizeof(wcfg.ip), "%s", ip);
                        }
                        char ss[APP_WIFI_SSID_MAX];
                        if (app_cmd_get_str(wobj, "sta_ssid", ss, sizeof(ss))) {
                            snprintf(wcfg.sta_ssid, sizeof(wcfg.sta_ssid), "%s", ss);
                        }
                        /*
                         * 密码: 仅在显式提供时覆盖。
                         *
                         * 普通导出不含密码，因此保持设备当前密码；
                         * 带 secrets 的完整迁移导出会带上明文，此处即恢复。
                         */
                        char pw[APP_WIFI_PASS_MAX];
                        if (app_cmd_get_str(wobj, "password", pw, sizeof(pw))) {
                            snprintf(wcfg.password, sizeof(wcfg.password), "%s", pw);
                            pass_restored = true;
                        }
                        char spw[APP_WIFI_PASS_MAX];
                        if (app_cmd_get_str(wobj, "sta_password", spw, sizeof(spw))) {
                            snprintf(wcfg.sta_password, sizeof(wcfg.sta_password), "%s", spw);
                            pass_restored = true;
                        }

                        if (app_wifi_set_cfg(&wcfg) == ESP_OK) {
                            wifi_applied = true;
                        }
                    }
                    free(wobj);
                }
            }
        }
    }

    snprintf(out, out_len,
             "{\"ok\":true,\"leds\":%u,\"wifi\":%s,\"passwords\":%s}",
             (unsigned)led_applied, wifi_applied ? "true" : "false",
             pass_restored ? "true" : "false");
    return ESP_OK;
}

/* ============================================================================
 * 命令分发
 * ========================================================================== */

esp_err_t app_cmd_execute(const char *json, char *out_buf, size_t out_len)
{
    if (json == NULL || out_buf == NULL || out_len < 64) {
        return ESP_ERR_INVALID_ARG;
    }

    char cmd[24];
    if (!app_cmd_get_str(json, "cmd", cmd, sizeof(cmd))) {
        return reply_error(out_buf, out_len, "缺少 cmd 字段");
    }

    ESP_LOGD(TAG, "执行命令: %s", cmd);

    /* --- 灯珠控制 --- */
    if (strcmp(cmd, "led.set") == 0) {
        return cmd_led_set(json, out_buf, out_len);
    }
    if (strcmp(cmd, "led.effect") == 0) {
        return cmd_led_effect(json, out_buf, out_len);
    }
    if (strcmp(cmd, "led.brightness") == 0) {
        return cmd_led_brightness(json, out_buf, out_len);
    }
    if (strcmp(cmd, "led.enable") == 0) {
        return cmd_led_enable(json, out_buf, out_len);
    }
    if (strcmp(cmd, "led.sequence") == 0) {
        return cmd_led_sequence(json, out_buf, out_len);
    }

    /* --- 全局灯控 --- */
    if (strcmp(cmd, "all") == 0) {
        return cmd_all(json, out_buf, out_len);
    }
    if (strcmp(cmd, "brightness") == 0) {
        return cmd_brightness(json, out_buf, out_len);
    }
    if (strcmp(cmd, "effect") == 0) {
        return cmd_effect(json, out_buf, out_len);
    }
    if (strcmp(cmd, "off") == 0) {
        return cmd_off(out_buf, out_len);
    }
    if (strcmp(cmd, "freq") == 0) {
        return cmd_freq(json, out_buf, out_len);
    }

    /* --- 状态 --- */
    if (strcmp(cmd, "status") == 0) {
        return cmd_status(out_buf, out_len);
    }

    /* --- WiFi --- */
    if (strcmp(cmd, "wifi.get") == 0) {
        return cmd_wifi_get(out_buf, out_len);
    }
    if (strcmp(cmd, "wifi.set") == 0) {
        return cmd_wifi_set(json, out_buf, out_len);
    }
    if (strcmp(cmd, "wifi.scan") == 0) {
        return cmd_wifi_scan(out_buf, out_len);
    }
    if (strcmp(cmd, "wifi.reconnect") == 0) {
        return cmd_wifi_reconnect(out_buf, out_len);
    }
    if (strcmp(cmd, "wifi.enable") == 0) {
        return cmd_wifi_enable(json, out_buf, out_len);
    }

    /* --- 蓝牙 --- */
    if (strcmp(cmd, "ble.enable") == 0) {
        return cmd_ble_enable(json, out_buf, out_len);
    }
    if (strcmp(cmd, "ble.pin") == 0) {
        return cmd_ble_pin(json, out_buf, out_len);
    }
    if (strcmp(cmd, "wifi.reset") == 0) {
        /*
         * 恢复默认 WiFi 配置。
         *
         * 与 HTTP 版本一致: 清除 NVS 用户配置后重启，让默认值生效。
         */
        esp_err_t e = app_wifi_reset_cfg();
        if (e != ESP_OK) {
            return reply_error(out_buf, out_len, esp_err_to_name(e));
        }
        snprintf(out_buf, out_len, "{\"ok\":true,\"reset\":true,\"reboot\":true}");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
        return ESP_OK;   /* 不会执行到 */
    }

    /* --- 配置导入 / 导出 --- */
    if (strcmp(cmd, "config.export") == 0) {
        return cmd_config_export(json, out_buf, out_len);
    }
    if (strcmp(cmd, "config.import") == 0) {
        return cmd_config_import(json, out_buf, out_len);
    }

    /*
     * 重启类命令。
     *
     * 先把响应写进缓冲区 (调用方负责发出去)，再延迟重启。
     * 延迟 500 ms 是为了让传输层有时间把响应送达 —— BLE 的
     * notify 尤其需要这点时间。
     */
    if (strcmp(cmd, "reboot") == 0) {
        snprintf(out_buf, out_len, "{\"ok\":true,\"action\":\"reboot\"}");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
        return ESP_OK;   /* 不会执行到 */
    }

    if (strcmp(cmd, "upgrade") == 0) {
        snprintf(out_buf, out_len, "{\"ok\":true,\"action\":\"upgrade\"}");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_err_t err = iap_user_request_download();
        ESP_LOGE(TAG, "请求下载失败: %s", esp_err_to_name(err));
        return ESP_OK;
    }

    return reply_error(out_buf, out_len, "未知命令");
}
