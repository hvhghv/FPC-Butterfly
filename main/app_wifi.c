/*
 * 蝴蝶板灯控应用 — WiFi SoftAP 模块实现
 *
 * 【v5 架构下的参数来源】
 *
 * 用户程序**不能**访问 iap_cfg 配置区 (位于 IDF flash 写保护区内，
 * 且是 IAP 私有数据)，因此网络参数只能:
 *   1. 从 RTC RAM 启动参数获取 (iap_boot_param_t.reserved1 预留了
 *      配置子集的位置，待 IAP 侧填入)
 *   2. 否则使用本模块内置默认值 (APP_WIFI_DEFAULT_*)
 *
 * 当前 v5 的 iap_boot_param_t 只传启动决策，不含 WiFi 配置，
 * 因此实际使用内置默认值。
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "iap_user_api.h"
#include "iap_boot_param.h"
#include "app_wifi.h"

static const char *TAG = "app_wifi";

/** NVS 命名空间 (存放 AP 配置) */
#define WIFI_NVS_NAMESPACE  "wificfg"

/** NVS 键名 */
#define WIFI_NVS_KEY_MODE   "mode"
#define WIFI_NVS_KEY_SSID   "ssid"
#define WIFI_NVS_KEY_PASS   "pass"
#define WIFI_NVS_KEY_CHAN   "chan"
#define WIFI_NVS_KEY_MAXCN  "maxcn"
#define WIFI_NVS_KEY_HIDDEN "hidden"
#define WIFI_NVS_KEY_IP     "ip"
#define WIFI_NVS_KEY_STASSID "stassid"
#define WIFI_NVS_KEY_STAPASS "stapass"
#define WIFI_NVS_KEY_STASTATIC "stastat"
#define WIFI_NVS_KEY_STAIP   "staip"
#define WIFI_NVS_KEY_STAMASK "stamask"
#define WIFI_NVS_KEY_STAGW   "stagw"
#define WIFI_NVS_KEY_STADNS  "stadns"

/*
 * 芯片能力检查: WiFi
 *
 * ESP32-H2 等芯片没有 WiFi 硬件，esp_wifi.h 中的配置宏不存在，
 * 此时整个模块退化为空实现。
 */
#if !SOC_WIFI_SUPPORTED

esp_err_t app_wifi_start(void)              { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t app_wifi_start_nonblocking(void)  { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t app_wifi_stop(void)               { return ESP_OK; }
const char *app_wifi_get_ip(void)           { return "0.0.0.0"; }
const char *app_wifi_get_ssid(void)         { return ""; }
uint8_t app_wifi_get_sta_count(void)        { return 0; }
esp_netif_t *app_wifi_get_netif(void)       { return NULL; }
esp_err_t app_wifi_get_cfg(app_wifi_cfg_t *cfg) { (void)cfg; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t app_wifi_set_cfg(const app_wifi_cfg_t *cfg) { (void)cfg; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t app_wifi_reset_cfg(void)          { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t app_wifi_get_sta_status(app_wifi_sta_status_t *st) { (void)st; return ESP_ERR_NOT_SUPPORTED; }
uint8_t app_wifi_get_mode(void)             { return APP_WIFI_MODE_AP; }
esp_err_t app_wifi_sta_reconnect(void)      { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t app_wifi_scan(app_wifi_ap_info_t *list, uint8_t max, uint8_t *found)
{
    (void)list; (void)max;
    if (found) *found = 0;
    return ESP_ERR_NOT_SUPPORTED;
}

#else  /* SOC_WIFI_SUPPORTED */

#define WIFI_AP_STARTED_BIT   BIT0
#define WIFI_STA_GOT_IP_BIT   BIT1

/** STA 断线重连最大次数 (超过后停止重试，避免无意义地耗电) */
#define WIFI_STA_MAX_RETRY    8

/** STA 重连间隔 (毫秒) */
#define WIFI_STA_RETRY_MS     3000

/** AP 网卡句柄 */
static esp_netif_t        *s_ap_netif = NULL;

/** STA 网卡句柄 */
static esp_netif_t        *s_sta_netif = NULL;

/** 事件组: 等待 AP 启动完成 / STA 拿到 IP */
static EventGroupHandle_t  s_events   = NULL;

/** 当前 AP 的 IP 字符串 */
static char                s_ip[16]   = "0.0.0.0";

/** 当前 AP 的 SSID */
static char                s_ssid[33] = APP_WIFI_DEFAULT_SSID;

/** 已接入 AP 的客户端数量 */
static volatile uint8_t    s_sta_count = 0;

/** 是否已启动 */
static bool                s_started  = false;

/** 当前生效的配置 (启动时从 NVS 加载，或由 set_cfg 更新) */
static app_wifi_cfg_t      s_cfg;

/** 配置是否已从 NVS 加载过 (避免重复读 flash) */
static bool                s_cfg_loaded = false;

/* --- STA 运行状态 (由事件处理函数更新) --- */
static volatile bool       s_sta_connected = false;   /*!< 是否已连上路由器 */
static volatile int8_t     s_sta_rssi      = 0;       /*!< 信号强度 dBm */
static char                s_sta_ip[16]    = "0.0.0.0"; /*!< 路由器分配的 IP */
static char                s_sta_gw[16]    = "0.0.0.0"; /*!< 网关 */
static volatile uint8_t    s_sta_retry     = 0;       /*!< 已重试次数 */

/* -------------------------------------------------------------------------- */
/* 事件处理                                                                    */
/* -------------------------------------------------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    /* --- IP 事件: STA 拿到地址 --- */
    if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = event_data;
        s_sta_connected = true;
        s_sta_retry     = 0;
        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&evt->ip_info.ip));
        snprintf(s_sta_gw, sizeof(s_sta_gw), IPSTR, IP2STR(&evt->ip_info.gw));

        ESP_LOGI(TAG, "STA 已连接路由器，获取 IP: %s (网关 %s)",
                 s_sta_ip, s_sta_gw);

        if (s_events) {
            xEventGroupSetBits(s_events, WIFI_STA_GOT_IP_BIT);
        }
        return;
    }

    if (base != WIFI_EVENT) {
        return;
    }

    switch (event_id) {
    case WIFI_EVENT_AP_START:
        ESP_LOGI(TAG, "AP 已启动");
        if (s_events) {
            xEventGroupSetBits(s_events, WIFI_AP_STARTED_BIT);
        }
        break;

    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *evt = event_data;
        if (s_sta_count < 0xFF) {
            s_sta_count++;
        }
        ESP_LOGI(TAG, "客户端接入: " MACSTR " (当前 %u 个)",
                 MAC2STR(evt->mac), s_sta_count);
        break;
    }

    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *evt = event_data;
        if (s_sta_count > 0) {
            s_sta_count--;
        }
        ESP_LOGI(TAG, "客户端断开: " MACSTR " (剩余 %u 个)",
                 MAC2STR(evt->mac), s_sta_count);
        break;
    }

    /* --- STA 事件 --- */
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "STA 已启动，开始连接路由器");
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "STA 已关联 AP，等待分配 IP...");
        break;

    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *evt = event_data;
        s_sta_connected = false;
        s_sta_rssi      = 0;
        snprintf(s_sta_ip, sizeof(s_sta_ip), "0.0.0.0");
        snprintf(s_sta_gw, sizeof(s_sta_gw), "0.0.0.0");

        if (s_events) {
            xEventGroupClearBits(s_events, WIFI_STA_GOT_IP_BIT);
        }

        /*
         * 断线重连。
         *
         * 限制重试次数，避免路由器不在时无限重连耗电 —— 也避免刷屏日志。
         * 用户可随时通过 /api/wifi/reconnect 手动重试。
         */
        if (s_sta_retry < WIFI_STA_MAX_RETRY) {
            s_sta_retry++;
            ESP_LOGW(TAG, "STA 断开 (原因 %u)，%d ms 后重试 (%u/%u)",
                     (unsigned)evt->reason, WIFI_STA_RETRY_MS,
                     s_sta_retry, WIFI_STA_MAX_RETRY);
            vTaskDelay(pdMS_TO_TICKS(WIFI_STA_RETRY_MS));
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "STA 断开 (原因 %u)，已达最大重试次数 %u，停止重连",
                     (unsigned)evt->reason, WIFI_STA_MAX_RETRY);
            ESP_LOGW(TAG, "可访问热点页面点击「重连」或调用 "
                          "POST /api/wifi/reconnect 手动重试");
        }
        break;
    }

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* 参数加载 / 保存                                                             */
/* -------------------------------------------------------------------------- */

/** 把配置填充为内置默认值 */
static void cfg_set_defaults(app_wifi_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->mode = APP_WIFI_MODE_AP;   /* 默认仅热点，最安全 */
    snprintf(cfg->ssid, sizeof(cfg->ssid), "%s", APP_WIFI_DEFAULT_SSID);
    snprintf(cfg->password, sizeof(cfg->password), "%s", APP_WIFI_DEFAULT_PASSWORD);
    cfg->channel  = APP_WIFI_DEFAULT_CHANNEL;
    cfg->max_conn = APP_WIFI_DEFAULT_MAX_CONN;
    cfg->hidden   = false;
    snprintf(cfg->ip, sizeof(cfg->ip), "192.168.4.1");
    /* STA 默认空，即未配置路由器；默认用 DHCP */
    cfg->sta_static_ip = false;
}

/**
 * @brief 从 RTC RAM 启动参数读取网络配置
 *
 * v5 架构下用户程序**不能**访问 iap_cfg 配置区 (位于 flash 写保护区内)，
 * 因此 WiFi 参数只能从 RTC RAM 的启动参数中获取。
 *
 * 当前 v5 的 iap_boot_param_t 只传启动决策，不含 WiFi 配置，
 * 因此这里始终返回 false，由调用者使用 NVS 或内置默认值。
 *
 * @param[out] cfg 输出配置 (调用前应先填好默认值)
 * @return true 表示成功从启动参数读取
 */
static bool load_net_params_from_boot_param(app_wifi_cfg_t *cfg)
{
    (void)cfg;

    static iap_user_boot_param_t bp;

    if (iap_user_boot_param_read(&bp) != ESP_OK) {
        return false;
    }

    /*
     * v5 的 iap_boot_param_t 目前只含启动决策 (boot_target / user_addr 等)，
     * 不含 WiFi 配置。配置子集预留在 reserved1[64] 中，待 IAP 侧填入后
     * 可在此解析。
     */
    ESP_LOGD(TAG, "RTC RAM 启动参数有效 (target=%u)，但 v5 暂不含网络配置",
             (unsigned)bp.boot_target);
    return false;
}

/** 从 NVS 读取单个字符串键 (不存在时保持原值) */
static void nvs_read_str(nvs_handle_t h, const char *key,
                         char *out, size_t out_len)
{
    size_t len = out_len;
    esp_err_t err = nvs_get_str(h, key, out, &len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "读取 NVS 键 %s 失败: %s", key, esp_err_to_name(err));
    }
    /* 读取失败时 out 保持调用者预设的默认值 */
}

/**
 * @brief 从 NVS 加载 AP 配置
 *
 * 未配置过的键保持调用者预设的默认值，因此首次启动即为默认配置。
 *
 * @param[out] cfg 输出配置 (调用前应先填好默认值)
 * @return true 表示至少读到了部分用户配置
 */
static bool load_cfg_from_nvs(app_wifi_cfg_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS 中无 WiFi 配置，使用内置默认值");
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "打开 NVS 命名空间失败: %s，使用内置默认值",
                 esp_err_to_name(err));
        return false;
    }

    nvs_read_str(h, WIFI_NVS_KEY_SSID, cfg->ssid, sizeof(cfg->ssid));
    nvs_read_str(h, WIFI_NVS_KEY_PASS, cfg->password, sizeof(cfg->password));
    nvs_read_str(h, WIFI_NVS_KEY_IP,   cfg->ip,   sizeof(cfg->ip));
    nvs_read_str(h, WIFI_NVS_KEY_STASSID, cfg->sta_ssid, sizeof(cfg->sta_ssid));
    nvs_read_str(h, WIFI_NVS_KEY_STAPASS, cfg->sta_password, sizeof(cfg->sta_password));
    nvs_read_str(h, WIFI_NVS_KEY_STAIP,   cfg->sta_ip,   sizeof(cfg->sta_ip));
    nvs_read_str(h, WIFI_NVS_KEY_STAMASK, cfg->sta_mask, sizeof(cfg->sta_mask));
    nvs_read_str(h, WIFI_NVS_KEY_STAGW,   cfg->sta_gw,   sizeof(cfg->sta_gw));
    nvs_read_str(h, WIFI_NVS_KEY_STADNS,  cfg->sta_dns,  sizeof(cfg->sta_dns));

    uint8_t u8 = 0;
    if (nvs_get_u8(h, WIFI_NVS_KEY_MODE, &u8) == ESP_OK) {
        cfg->mode = u8;
    }
    if (nvs_get_u8(h, WIFI_NVS_KEY_STASTATIC, &u8) == ESP_OK) {
        cfg->sta_static_ip = (u8 != 0);
    }
    if (nvs_get_u8(h, WIFI_NVS_KEY_CHAN, &u8) == ESP_OK) {
        cfg->channel = u8;
    }
    if (nvs_get_u8(h, WIFI_NVS_KEY_MAXCN, &u8) == ESP_OK) {
        cfg->max_conn = u8;
    }
    if (nvs_get_u8(h, WIFI_NVS_KEY_HIDDEN, &u8) == ESP_OK) {
        cfg->hidden = (u8 != 0);
    }

    nvs_close(h);
    ESP_LOGI(TAG, "已从 NVS 加载 WiFi 配置");
    return true;
}

/**
 * @brief 把 AP 配置写入 NVS
 *
 * @param[in] cfg 要保存的配置
 * @return ESP_OK 成功
 */
static esp_err_t save_cfg_to_nvs(const app_wifi_cfg_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "打开 NVS 命名空间失败: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(h, WIFI_NVS_KEY_SSID, cfg->ssid);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_PASS, cfg->password);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_IP,   cfg->ip);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_STASSID, cfg->sta_ssid);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_STAPASS, cfg->sta_password);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_STAIP,   cfg->sta_ip);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_STAMASK, cfg->sta_mask);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_STAGW,   cfg->sta_gw);
    if (err == ESP_OK) err = nvs_set_str(h, WIFI_NVS_KEY_STADNS,  cfg->sta_dns);
    if (err == ESP_OK) err = nvs_set_u8(h, WIFI_NVS_KEY_STASTATIC,
                                        cfg->sta_static_ip ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u8(h, WIFI_NVS_KEY_MODE,   cfg->mode);
    if (err == ESP_OK) err = nvs_set_u8(h, WIFI_NVS_KEY_CHAN,   cfg->channel);
    if (err == ESP_OK) err = nvs_set_u8(h, WIFI_NVS_KEY_MAXCN,  cfg->max_conn);
    if (err == ESP_OK) err = nvs_set_u8(h, WIFI_NVS_KEY_HIDDEN,
                                        cfg->hidden ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写入 NVS 失败: %s", esp_err_to_name(err));
    }
    return err;
}

/* 前置声明: 定义在后面，但 apply_cfg_live() 与启动流程都要用 */
static esp_err_t apply_sta_ip_mode(const app_wifi_cfg_t *cfg);
static esp_err_t apply_ap_ip_mode(const app_wifi_cfg_t *cfg);

/**
 * @brief 确保 s_cfg 已加载 (首次调用时从 NVS 读取)
 */
static void ensure_cfg_loaded(void)
{
    if (s_cfg_loaded) {
        return;
    }

    cfg_set_defaults(&s_cfg);

    /* 优先级: NVS 用户配置 > IAP 启动参数 > 内置默认值 */
    if (!load_cfg_from_nvs(&s_cfg)) {
        load_net_params_from_boot_param(&s_cfg);
    }

    s_cfg_loaded = true;
}

/* -------------------------------------------------------------------------- */
/* 配置读取 / 修改                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t app_wifi_get_cfg(app_wifi_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ensure_cfg_loaded();
    *cfg = s_cfg;
    return ESP_OK;
}

/** 校验 IPv4 字符串，合法时返回主机序地址，非法返回 0 */
static uint32_t parse_ipv4(const char *s)
{
    if (s == NULL) {
        return 0;
    }

    uint32_t parts[4];
    int n = 0;

    while (n < 4) {
        if (*s < '0' || *s > '9') {
            return 0;       /* 必须以数字开头 */
        }

        char *end = NULL;
        long v = strtol(s, &end, 10);
        if (end == s || v < 0 || v > 255) {
            return 0;
        }
        parts[n++] = (uint32_t)v;

        if (n == 4) {
            if (*end != '\0') {
                return 0;   /* 结尾必须干净 */
            }
            break;
        }
        if (*end != '.') {
            return 0;
        }
        s = end + 1;
    }

    return (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8) | parts[3];
}

/** 校验配置合法性 (按模式只校验启用部分的参数) */
static esp_err_t validate_cfg(const app_wifi_cfg_t *cfg)
{
    if (cfg->mode > APP_WIFI_MODE_APSTA) {
        ESP_LOGW(TAG, "WiFi 模式非法: %u (需 0=AP 1=STA 2=APSTA)", cfg->mode);
        return ESP_ERR_INVALID_ARG;
    }

    bool use_ap  = (cfg->mode == APP_WIFI_MODE_AP ||
                    cfg->mode == APP_WIFI_MODE_APSTA);
    bool use_sta = (cfg->mode == APP_WIFI_MODE_STA ||
                    cfg->mode == APP_WIFI_MODE_APSTA);

    /* --- AP 参数 --- */
    if (use_ap) {
        size_t ssid_len = strnlen(cfg->ssid, sizeof(cfg->ssid));
        if (ssid_len == 0 || ssid_len >= sizeof(cfg->ssid)) {
            ESP_LOGW(TAG, "AP SSID 长度非法: %u (需 1-32)", (unsigned)ssid_len);
            return ESP_ERR_INVALID_ARG;
        }

        size_t pass_len = strnlen(cfg->password, sizeof(cfg->password));
        if (pass_len != 0 &&
            (pass_len < 8 || pass_len >= sizeof(cfg->password))) {
            ESP_LOGW(TAG, "AP 密码长度非法: %u (需 8-63，或留空为开放网络)",
                     (unsigned)pass_len);
            return ESP_ERR_INVALID_ARG;
        }

        if (cfg->channel < 1 || cfg->channel > 13) {
            ESP_LOGW(TAG, "信道非法: %u (需 1-13)", cfg->channel);
            return ESP_ERR_INVALID_ARG;
        }

        if (cfg->max_conn < 1 || cfg->max_conn > 10) {
            ESP_LOGW(TAG, "最大连接数非法: %u (需 1-10)", cfg->max_conn);
            return ESP_ERR_INVALID_ARG;
        }

        uint32_t ip = parse_ipv4(cfg->ip);
        if (ip == 0) {
            ESP_LOGW(TAG, "AP IP 地址非法: '%s'", cfg->ip);
            return ESP_ERR_INVALID_ARG;
        }
        /*
         * AP 自身地址必须是网段的第 1 个可用地址 (x.y.z.1)，
         * 否则 DHCP 服务器的网关设置会与 AP 地址不一致。
         */
        if ((ip & 0xFF) != 1) {
            ESP_LOGW(TAG, "AP 地址必须以 .1 结尾 (如 192.168.4.1)");
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* --- STA 参数 --- */
    if (use_sta) {
        size_t s = strnlen(cfg->sta_ssid, sizeof(cfg->sta_ssid));
        if (s == 0 || s >= sizeof(cfg->sta_ssid)) {
            ESP_LOGW(TAG, "STA SSID 长度非法: %u (需 1-32)", (unsigned)s);
            return ESP_ERR_INVALID_ARG;
        }

        size_t p = strnlen(cfg->sta_password, sizeof(cfg->sta_password));
        if (p != 0 && (p < 8 || p >= sizeof(cfg->sta_password))) {
            ESP_LOGW(TAG, "STA 密码长度非法: %u (需 8-63，或留空为开放网络)",
                     (unsigned)p);
            return ESP_ERR_INVALID_ARG;
        }

        /* 静态 IP: 三项必填且合法，且 IP 不能与网关相同 */
        if (cfg->sta_static_ip) {
            uint32_t ip   = parse_ipv4(cfg->sta_ip);
            uint32_t mask = parse_ipv4(cfg->sta_mask);
            uint32_t gw   = parse_ipv4(cfg->sta_gw);

            if (ip == 0) {
                ESP_LOGW(TAG, "STA 静态 IP 非法: '%s'", cfg->sta_ip);
                return ESP_ERR_INVALID_ARG;
            }
            if (mask == 0) {
                ESP_LOGW(TAG, "STA 子网掩码非法: '%s'", cfg->sta_mask);
                return ESP_ERR_INVALID_ARG;
            }
            if (gw == 0) {
                ESP_LOGW(TAG, "STA 网关非法: '%s'", cfg->sta_gw);
                return ESP_ERR_INVALID_ARG;
            }
            if (ip == gw) {
                ESP_LOGW(TAG, "STA 静态 IP 不能与网关相同");
                return ESP_ERR_INVALID_ARG;
            }
            /*
             * IP 与网关必须在同一子网，否则设备无法与网关通信。
             * 判据: (ip & mask) == (gw & mask)
             */
            if ((ip & mask) != (gw & mask)) {
                ESP_LOGW(TAG, "STA 静态 IP 与网关不在同一子网: "
                              "%s / %s / %s",
                         cfg->sta_ip, cfg->sta_mask, cfg->sta_gw);
                return ESP_ERR_INVALID_ARG;
            }
            /* DNS 可留空；非空时必须合法 */
            if (cfg->sta_dns[0] != '\0' && parse_ipv4(cfg->sta_dns) == 0) {
                ESP_LOGW(TAG, "STA DNS 非法: '%s'", cfg->sta_dns);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    return ESP_OK;
}

/** 把配置应用到正在运行的 WiFi (热生效) */
static esp_err_t apply_cfg_live(const app_wifi_cfg_t *cfg)
{
    esp_err_t err;

    /* --- 1. 切换工作模式 --- */
    wifi_mode_t mode;
    switch (cfg->mode) {
    case APP_WIFI_MODE_STA:   mode = WIFI_MODE_STA;   break;
    case APP_WIFI_MODE_APSTA: mode = WIFI_MODE_APSTA; break;
    case APP_WIFI_MODE_AP:
    default:                  mode = WIFI_MODE_AP;    break;
    }

    err = esp_wifi_set_mode(mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "切换 WiFi 模式失败: %s", esp_err_to_name(err));
        return err;
    }

    bool use_ap  = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
    bool use_sta = (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA);

    /* --- 2. AP 配置 --- */
    if (use_ap) {
        wifi_config_t ap_cfg = {0};

        size_t ssid_len = strlen(cfg->ssid);
        memcpy(ap_cfg.ap.ssid, cfg->ssid, ssid_len);
        ap_cfg.ap.ssid_len = (uint8_t)ssid_len;

        size_t pass_len = strlen(cfg->password);
        if (pass_len >= 8) {
            memcpy(ap_cfg.ap.password, cfg->password, pass_len);
            ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
        } else {
            ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
        }

        ap_cfg.ap.channel          = cfg->channel;
        ap_cfg.ap.max_connection   = cfg->max_conn;
        ap_cfg.ap.ssid_hidden      = cfg->hidden ? 1 : 0;
        ap_cfg.ap.beacon_interval  = 100;
        ap_cfg.ap.pmf_cfg.required = false;

        err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "应用 AP 配置失败: %s", esp_err_to_name(err));
            return err;
        }

        /*
         * 更新 AP 自身 IP (网段变化时 DHCP 池也要跟着变)。
         *
         * 此处是热更新路径，WiFi 已启动、网卡已就绪，
         * 因此可以直接应用 IP 与 DHCP 服务器。
         */
        apply_ap_ip_mode(cfg);

        snprintf(s_ssid, sizeof(s_ssid), "%s", cfg->ssid);
        snprintf(s_ip, sizeof(s_ip), "%s", cfg->ip);
    }

    /* --- 3. STA 配置 --- */
    if (use_sta) {
        wifi_config_t sta_cfg = {0};

        size_t s = strlen(cfg->sta_ssid);
        memcpy(sta_cfg.sta.ssid, cfg->sta_ssid, s);

        size_t p = strlen(cfg->sta_password);
        if (p > 0) {
            memcpy(sta_cfg.sta.password, cfg->sta_password, p);
        }

        /* 允许所有认证方式，由驱动自动协商 */
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        sta_cfg.sta.pmf_cfg.capable    = true;
        sta_cfg.sta.pmf_cfg.required   = false;

        err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "应用 STA 配置失败: %s", esp_err_to_name(err));
            return err;
        }

        /* 应用 IP 获取方式 (DHCP / 静态) */
        apply_sta_ip_mode(cfg);

        /* 重置重试计数并重新连接 */
        s_sta_retry = 0;
        esp_wifi_disconnect();
        esp_wifi_connect();
        ESP_LOGI(TAG, "STA 正在连接路由器 '%s'...", cfg->sta_ssid);
    }

    return ESP_OK;
}

esp_err_t app_wifi_get_sta_status(app_wifi_sta_status_t *st)
{
    if (st == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ensure_cfg_loaded();

    memset(st, 0, sizeof(*st));
    st->enabled   = (s_cfg.mode == APP_WIFI_MODE_STA ||
                     s_cfg.mode == APP_WIFI_MODE_APSTA);
    st->static_ip = s_cfg.sta_static_ip;
    snprintf(st->ssid, sizeof(st->ssid), "%s", s_cfg.sta_ssid);

    if (s_started && st->enabled) {
        st->connected = s_sta_connected;

        /*
         * IP / 掩码 / 网关 / DNS 直接读网卡，而不是用事件里缓存的字符串。
         *
         * 静态 IP 模式下没有 DHCP 事件，缓存值可能为空；
         * 且静态 IP 在连接前就已生效，读网卡才能反映真实状态。
         */
        if (s_sta_netif != NULL) {
            esp_netif_ip_info_t ip_info = {0};
            if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK &&
                ip_info.ip.addr != 0) {
                snprintf(st->ip, sizeof(st->ip), IPSTR, IP2STR(&ip_info.ip));
                snprintf(st->gw, sizeof(st->gw), IPSTR, IP2STR(&ip_info.gw));
                snprintf(st->mask, sizeof(st->mask), IPSTR, IP2STR(&ip_info.netmask));
            }

            esp_netif_dns_info_t dns = {0};
            if (esp_netif_get_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN,
                                       &dns) == ESP_OK &&
                dns.ip.u_addr.ip4.addr != 0) {
                snprintf(st->dns, sizeof(st->dns), IPSTR,
                         IP2STR(&dns.ip.u_addr.ip4));
            }
        }

        /* 网卡没给出有效 IP 时退回事件缓存值 */
        if (st->ip[0] == '\0') {
            snprintf(st->ip, sizeof(st->ip), "%s", s_sta_ip);
        }
        if (st->gw[0] == '\0') {
            snprintf(st->gw, sizeof(st->gw), "%s", s_sta_gw);
        }

        /* RSSI / 信道 / BSSID 只在连接状态下有意义 */
        if (s_sta_connected) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                s_sta_rssi = ap_info.rssi;
                st->channel = ap_info.primary;
                memcpy(st->bssid, ap_info.bssid, sizeof(st->bssid));
            }
        }
        st->rssi = s_sta_rssi;
    } else {
        snprintf(st->ip, sizeof(st->ip), "0.0.0.0");
        snprintf(st->gw, sizeof(st->gw), "0.0.0.0");
    }

    return ESP_OK;
}

esp_err_t app_wifi_sta_reconnect(void)
{
    ensure_cfg_loaded();

    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cfg.mode != APP_WIFI_MODE_STA && s_cfg.mode != APP_WIFI_MODE_APSTA) {
        ESP_LOGW(TAG, "当前模式未启用 STA，无法重连");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cfg.sta_ssid[0] == '\0') {
        ESP_LOGW(TAG, "未配置路由器 SSID，无法重连");
        return ESP_ERR_INVALID_STATE;
    }

    /* 重置计数，让重连逻辑重新开始 */
    s_sta_retry = 0;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t err = esp_wifi_connect();

    ESP_LOGI(TAG, "手动触发 STA 重连: %s", esp_err_to_name(err));
    return err;
}

esp_err_t app_wifi_scan(app_wifi_ap_info_t *list, uint8_t max, uint8_t *found)
{
    if (list == NULL || found == NULL || max == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    *found = 0;

    /*
     * 扫描参数: 每信道停留 120 ms。
     * 时间太短会漏掉弱信号 AP，太长则用户等待过久。
     */
    wifi_scan_config_t scan_cfg = {
        .ssid        = NULL,
        .bssid       = NULL,
        .channel     = 0,       /* 0 = 扫描全部信道 */
        .show_hidden = false,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time   = { .active = { .min = 120, .max = 200 } },
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);   /* true = 阻塞 */
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "扫描失败: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t ap_num = max;
    wifi_ap_record_t records[APP_WIFI_SCAN_MAX];
    if (ap_num > APP_WIFI_SCAN_MAX) {
        ap_num = APP_WIFI_SCAN_MAX;
    }

    err = esp_wifi_scan_get_ap_records(&ap_num, records);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "读取扫描结果失败: %s", esp_err_to_name(err));
        return err;
    }

    for (uint16_t i = 0; i < ap_num; i++) {
        /* 跳过空 SSID (隐藏热点) */
        if (records[i].ssid[0] == '\0') {
            continue;
        }
        snprintf(list[*found].ssid, sizeof(list[*found].ssid), "%s",
                 (const char *)records[i].ssid);
        list[*found].rssi     = records[i].rssi;
        list[*found].channel  = records[i].primary;
        list[*found].authmode = (uint8_t)records[i].authmode;
        (*found)++;
    }

    ESP_LOGI(TAG, "扫描完成，找到 %u 个热点", (unsigned)*found);
    return ESP_OK;
}

esp_err_t app_wifi_set_cfg(const app_wifi_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = validate_cfg(cfg);
    if (err != ESP_OK) {
        return err;
    }

    ensure_cfg_loaded();

    if (!s_started) {
        ESP_LOGW(TAG, "WiFi 未启动，配置已保存但需重启后生效");
        s_cfg = *cfg;
        s_cfg_loaded = true;
        return save_cfg_to_nvs(cfg);
    }

    err = apply_cfg_live(cfg);
    if (err != ESP_OK) {
        return err;
    }

    s_cfg = *cfg;

    /* 先热生效再落盘: 落盘失败不影响本次运行，但会告警 */
    esp_err_t nvs_err = save_cfg_to_nvs(cfg);
    if (nvs_err != ESP_OK) {
        ESP_LOGW(TAG, "配置已生效但未能持久化，重启后将恢复旧值");
    }

    ESP_LOGI(TAG, "WiFi 配置已更新: SSID='%s' 信道=%u 认证=%s 隐藏=%s 地址=%s",
             cfg->ssid, cfg->channel,
             (strlen(cfg->password) >= 8) ? "WPA2" : "开放",
             cfg->hidden ? "是" : "否", cfg->ip);

    return ESP_OK;
}

esp_err_t app_wifi_reset_cfg(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_erase_all(h);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "清除 NVS WiFi 配置失败: %s", esp_err_to_name(err));
        return err;
    }

    cfg_set_defaults(&s_cfg);
    s_cfg_loaded = true;

    ESP_LOGI(TAG, "WiFi 配置已恢复默认 (需重启或重新应用后生效)");
    return ESP_OK;
}

/**
 * @brief 应用 STA 的 IP 获取方式 (DHCP 或静态)
 *
 * ESP-IDF 的 DHCP 客户端与静态 IP 互斥: 设置静态 IP 前必须先停掉
 * DHCP 客户端，否则 esp_netif_set_ip_info() 会被拒绝 (返回
 * ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED)。
 *
 * @param cfg 配置
 * @return ESP_OK 成功
 */
static esp_err_t apply_sta_ip_mode(const app_wifi_cfg_t *cfg)
{
    if (s_sta_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (cfg->sta_static_ip) {
        esp_netif_ip_info_t ip_info = {0};
        ip_info.ip.addr      = htonl(parse_ipv4(cfg->sta_ip));
        ip_info.netmask.addr = htonl(parse_ipv4(cfg->sta_mask));
        ip_info.gw.addr      = htonl(parse_ipv4(cfg->sta_gw));

        /* 必须先停 DHCP 客户端，否则 set_ip_info 会失败 */
        esp_netif_dhcpc_stop(s_sta_netif);

        esp_err_t err = esp_netif_set_ip_info(s_sta_netif, &ip_info);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "设置 STA 静态 IP 失败: %s", esp_err_to_name(err));
            return err;
        }

        /* DNS 可选；留空时用网关作为 DNS */
        if (cfg->sta_dns[0] != '\0') {
            esp_netif_dns_info_t dns = {0};
            dns.ip.u_addr.ip4.addr = htonl(parse_ipv4(cfg->sta_dns));
            dns.ip.type = ESP_IPADDR_TYPE_V4;
            esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns);
        } else {
            esp_netif_dns_info_t dns = {0};
            dns.ip.u_addr.ip4.addr = htonl(parse_ipv4(cfg->sta_gw));
            dns.ip.type = ESP_IPADDR_TYPE_V4;
            esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns);
        }

        ESP_LOGI(TAG, "STA 静态 IP: %s/%s 网关 %s%s%s",
                 cfg->sta_ip, cfg->sta_mask, cfg->sta_gw,
                 cfg->sta_dns[0] ? " DNS " : "",
                 cfg->sta_dns[0] ? cfg->sta_dns : "");
    } else {
        /* 切回 DHCP */
        esp_netif_dhcpc_stop(s_sta_netif);
        esp_err_t err = esp_netif_dhcpc_start(s_sta_netif);
        if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
            ESP_LOGW(TAG, "启动 STA DHCP 客户端失败: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "STA 使用 DHCP 获取 IP");
    }

    return ESP_OK;
}

/**
 * @brief 应用 AP 的静态 IP 与 DHCP 服务器
 *
 * **必须在 esp_wifi_start() 之后调用**。
 *
 * 原因: esp_wifi_start() 会重新初始化 AP 网卡。若在此之前启停 DHCP
 * 服务器，网卡的子网掩码会被清空，导致:
 *
 *   dhcps: Illegal subnet mask.
 *   esp_netif_lwip: DHCP server cannot be started
 *
 * 客户端连上 AP 后拿不到 IP。
 *
 * @param cfg 配置
 * @return ESP_OK 成功
 */
static esp_err_t apply_ap_ip_mode(const app_wifi_cfg_t *cfg)
{
    if (s_ap_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t ip = parse_ipv4(cfg->ip);
    if (ip == 0) {
        return ESP_OK;
    }

    esp_netif_ip_info_t ip_info = {0};
    ip_info.ip.addr      = htonl(ip);
    ip_info.gw.addr      = htonl(ip);
    /*
     * 子网掩码 255.255.255.0。
     *
     * 注意字节序: lwIP 的 ip4_addr_t 以**网络序**存储，
     * 主机序的 255.255.255.0 = 0xFFFFFF00，因此必须 htonl(0xFFFFFF00)。
     * 若误写成 htonl(0x00FFFFFF)，网络序下会变成 0.255.255.255，
     * DHCP 服务器启动时报 "dhcps: Illegal subnet mask."。
     */
    ip_info.netmask.addr = htonl(0xFFFFFF00);

    /* 必须先停 DHCP 服务器，否则 set_ip_info 会被拒绝 */
    esp_netif_dhcps_stop(s_ap_netif);

    esp_err_t err = esp_netif_set_ip_info(s_ap_netif, &ip_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置 AP IP 失败: %s", esp_err_to_name(err));
        /* 仍尝试恢复 DHCP 服务器，避免客户端完全无法获取 IP */
        esp_netif_dhcps_start(s_ap_netif);
        return err;
    }

    err = esp_netif_dhcps_start(s_ap_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGW(TAG, "启动 AP DHCP 服务器失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "AP 地址已应用: %s/255.255.255.0", cfg->ip);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* 启动 / 停止                                                                 */
/* -------------------------------------------------------------------------- */

/** 内部启动实现，区别仅在于是否等待 AP 就绪 */
static esp_err_t wifi_start_internal(bool wait_ready);

esp_err_t app_wifi_start(void)
{
    return wifi_start_internal(true);
}

esp_err_t app_wifi_start_nonblocking(void)
{
    return wifi_start_internal(false);
}

static esp_err_t wifi_start_internal(bool wait_ready)
{
    /* --- 参数: NVS 用户配置 > IAP 启动参数 > 内置默认值 --- */
    ensure_cfg_loaded();

    bool use_ap  = (s_cfg.mode == APP_WIFI_MODE_AP ||
                    s_cfg.mode == APP_WIFI_MODE_APSTA);
    bool use_sta = (s_cfg.mode == APP_WIFI_MODE_STA ||
                    s_cfg.mode == APP_WIFI_MODE_APSTA);

    /* --- 网络栈 --- */
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init 失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "创建事件循环失败: %s", esp_err_to_name(err));
        return err;
    }

    if (s_events == NULL) {
        s_events = xEventGroupCreate();
        if (s_events == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    /*
     * 获取网卡句柄。
     *
     * 关键: IAP 在等待窗口内可能已经启动过 WiFi（等待触发路径），
     * 此时 WIFI_AP_DEF 网卡已存在。直接调用
     * esp_netif_create_default_wifi_ap() 会因 if_key 重复而失败并触发断言:
     *
     *   assert failed: esp_netif_create_default_wifi_ap
     *   (netif)  wifi_default.c:408
     *
     * 因此先按 key 查找已有网卡，找到就复用，找不到才创建。
     * 同理，esp_wifi_init() 在已初始化时返回 ESP_ERR_INVALID_STATE，
     * 也按"复用"处理。
     */
    if (use_ap && s_ap_netif == NULL) {
        s_ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (s_ap_netif != NULL) {
            ESP_LOGI(TAG, "复用已存在的 AP 网卡 (WIFI_AP_DEF)");
        }
    }

    if (use_ap && s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            ESP_LOGE(TAG, "创建 AP 网卡失败");
            return ESP_FAIL;
        }
    }

    if (use_sta && s_sta_netif == NULL) {
        s_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (s_sta_netif != NULL) {
            ESP_LOGI(TAG, "复用已存在的 STA 网卡 (WIFI_STA_DEF)");
        }
    }

    if (use_sta && s_sta_netif == NULL) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (s_sta_netif == NULL) {
            ESP_LOGE(TAG, "创建 STA 网卡失败");
            return ESP_FAIL;
        }
    }

    /* --- AP 静态 IP --- */
    /*
     * 注意: 这里**不**配置 AP IP，也不启停 DHCP 服务器。
     *
     * 原因: 此时 esp_wifi_start() 尚未调用，AP 网卡底层还没就绪。
     * 若此刻 esp_netif_dhcps_start()，随后 esp_wifi_start() 会重新
     * 初始化 AP 网卡，导致 DHCP 服务器的子网掩码被清空，启动时报:
     *
     *   dhcps: Illegal subnet mask.
     *   esp_netif_lwip: DHCP server cannot be started
     *
     * 结果客户端连上 AP 后拿不到 IP。因此 AP IP 与 DHCP 的配置统一
     * 放到 esp_wifi_start() 之后（见下面的 apply_ap_ip_mode()）。
     */

    /* --- WiFi 初始化 --- */
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        /* IAP 等待窗口内已初始化过 WiFi，直接复用 */
        ESP_LOGI(TAG, "WiFi 驱动已初始化，复用现有实例");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * 配置不写 NVS。
     *
     * 本模块自行用 "wificfg" 命名空间管理配置，若再让 WiFi 驱动
     * 把配置写进 NVS，会与我们的键混在一起且无法预测键名。
     * 因此保持 WIFI_STORAGE_RAM，持久化完全由 save_cfg_to_nvs() 负责。
     */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 WiFi 事件失败: %s", esp_err_to_name(err));
        return err;
    }

    /* STA 拿到 IP 的事件单独注册 (IP_EVENT 是另一个事件基) */
    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 IP 事件失败: %s", esp_err_to_name(err));
        return err;
    }

    /* --- 工作模式 --- */
    wifi_mode_t mode;
    switch (s_cfg.mode) {
    case APP_WIFI_MODE_STA:   mode = WIFI_MODE_STA;   break;
    case APP_WIFI_MODE_APSTA: mode = WIFI_MODE_APSTA; break;
    case APP_WIFI_MODE_AP:
    default:                  mode = WIFI_MODE_AP;    break;
    }

    err = esp_wifi_set_mode(mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置 WiFi 模式失败: %s", esp_err_to_name(err));
        return err;
    }

    /* --- AP 配置 --- */
    if (use_ap) {
        wifi_config_t ap_cfg = {0};
        size_t ssid_len = strlen(s_cfg.ssid);
        if (ssid_len > sizeof(ap_cfg.ap.ssid)) {
            ssid_len = sizeof(ap_cfg.ap.ssid);
        }
        memcpy(ap_cfg.ap.ssid, s_cfg.ssid, ssid_len);
        ap_cfg.ap.ssid_len = (uint8_t)ssid_len;

        size_t pass_len = strlen(s_cfg.password);
        if (pass_len >= 8 && pass_len <= 63) {
            memcpy(ap_cfg.ap.password, s_cfg.password, pass_len);
            ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
        } else {
            if (pass_len != 0) {
                ESP_LOGW(TAG, "AP 密码长度 %u 不合法 (需 8-63 字符)，改为开放网络",
                         (unsigned)pass_len);
            }
            ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
        }

        ap_cfg.ap.channel          = s_cfg.channel;
        ap_cfg.ap.max_connection   = s_cfg.max_conn;
        ap_cfg.ap.ssid_hidden      = s_cfg.hidden ? 1 : 0;
        ap_cfg.ap.beacon_interval  = 100;
        ap_cfg.ap.pmf_cfg.required = false;

        err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "设置 AP 配置失败: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* --- STA 配置 --- */
    if (use_sta) {
        wifi_config_t sta_cfg = {0};
        size_t s = strlen(s_cfg.sta_ssid);
        if (s > sizeof(sta_cfg.sta.ssid)) {
            s = sizeof(sta_cfg.sta.ssid);
        }
        memcpy(sta_cfg.sta.ssid, s_cfg.sta_ssid, s);

        size_t p = strlen(s_cfg.sta_password);
        if (p > 0) {
            if (p > sizeof(sta_cfg.sta.password)) {
                p = sizeof(sta_cfg.sta.password);
            }
            memcpy(sta_cfg.sta.password, s_cfg.sta_password, p);
        }

        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        sta_cfg.sta.pmf_cfg.capable    = true;
        sta_cfg.sta.pmf_cfg.required   = false;

        err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "设置 STA 配置失败: %s", esp_err_to_name(err));
            return err;
        }
    }

    /*
     * 启动 WiFi。
     *
     * 若 IAP 已在等待窗口内启动过 WiFi，esp_wifi_start() 会返回
     * ESP_ERR_WIFI_STATE (已启动)。此时配置已通过 set_config 生效，
     * 视为成功继续。
     */
    err = esp_wifi_start();
    if (err == ESP_ERR_WIFI_STATE) {
        ESP_LOGI(TAG, "WiFi 已在运行，配置已更新");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动 WiFi 失败: %s", esp_err_to_name(err));
        return err;
    }

    s_started = true;
    snprintf(s_ssid, sizeof(s_ssid), "%s", s_cfg.ssid);

    /*
     * 应用 STA 的 IP 获取方式。
     *
     * 必须在 esp_wifi_start() 之后 —— 此时 STA 网卡才真正可用，
     * DHCP 客户端的启停才有意义。
     */
    if (use_sta) {
        apply_sta_ip_mode(&s_cfg);
    }

    /*
     * 应用 AP 的静态 IP 与 DHCP 服务器。
     *
     * 同样必须在 esp_wifi_start() 之后 —— 否则 AP 网卡的子网掩码
     * 会被重新初始化清空，DHCP 服务器启动失败 (Illegal subnet mask)。
     */
    if (use_ap) {
        apply_ap_ip_mode(&s_cfg);
    }

    static const char *mode_names[] = { "AP", "STA", "APSTA" };
    ESP_LOGI(TAG, "WiFi 已启动: 模式=%s", mode_names[s_cfg.mode]);

    if (use_ap) {
        ESP_LOGI(TAG, "  AP : SSID='%s' 信道=%u 认证=%s 隐藏=%s",
                 s_cfg.ssid, s_cfg.channel,
                 (strlen(s_cfg.password) >= 8) ? "WPA2" : "开放",
                 s_cfg.hidden ? "是" : "否");
    }
    if (use_sta) {
        ESP_LOGI(TAG, "  STA: 正在连接 '%s' (密码%s)",
                 s_cfg.sta_ssid,
                 (strlen(s_cfg.sta_password) > 0) ? "已设置" : "为空/开放");
    }

    /* 等待 AP 就绪 (阻塞模式) */
    if (wait_ready && use_ap) {
        EventBits_t bits = xEventGroupWaitBits(s_events, WIFI_AP_STARTED_BIT,
                                               pdFALSE, pdTRUE,
                                               pdMS_TO_TICKS(10000));
        if (!(bits & WIFI_AP_STARTED_BIT)) {
            ESP_LOGW(TAG, "等待 AP 启动超时");
        }
    }

    /*
     * 等待 STA 拿到 IP。
     *
     * 只等 5 秒 —— STA 连不上路由器不能阻塞启动流程，
     * 否则 AP 兜底通道也无法使用。超时后 STA 会在后台继续重连。
     */
    if (wait_ready && use_sta) {
        EventBits_t bits = xEventGroupWaitBits(s_events, WIFI_STA_GOT_IP_BIT,
                                               pdFALSE, pdTRUE,
                                               pdMS_TO_TICKS(5000));
        if (bits & WIFI_STA_GOT_IP_BIT) {
            ESP_LOGI(TAG, "  路由器已连接: http://%s/", s_sta_ip);
        } else {
            ESP_LOGW(TAG, "  STA 暂未连上路由器，将在后台继续重连");
            if (!use_ap) {
                ESP_LOGW(TAG, "  ⚠️ 当前为纯 STA 模式且未连上，"
                              "只能通过串口访问");
            }
        }
    }

    /* 记录 AP IP 字符串 */
    if (use_ap) {
        esp_netif_ip_info_t actual = {0};
        if (esp_netif_get_ip_info(s_ap_netif, &actual) == ESP_OK &&
            actual.ip.addr) {
            snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&actual.ip));
        }
    }

    if (wait_ready && use_ap) {
        ESP_LOGI(TAG, "  AP 地址: http://%s/", s_ip);
    }

    return ESP_OK;
}

esp_err_t app_wifi_stop(void)
{
    if (!s_started) {
        return ESP_OK;
    }

    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_ap_netif) {
        esp_netif_destroy_default_wifi(s_ap_netif);
        s_ap_netif = NULL;
    }
    if (s_sta_netif) {
        esp_netif_destroy_default_wifi(s_sta_netif);
        s_sta_netif = NULL;
    }

    s_started       = false;
    s_sta_count     = 0;
    s_sta_connected = false;
    s_sta_rssi      = 0;
    s_sta_retry     = 0;
    snprintf(s_ip, sizeof(s_ip), "0.0.0.0");
    snprintf(s_sta_ip, sizeof(s_sta_ip), "0.0.0.0");
    snprintf(s_sta_gw, sizeof(s_sta_gw), "0.0.0.0");

    ESP_LOGI(TAG, "WiFi 已停止");
    return ESP_OK;
}

const char *app_wifi_get_ip(void)
{
    return s_ip;
}

uint8_t app_wifi_get_mode(void)
{
    ensure_cfg_loaded();
    return s_cfg.mode;
}

const char *app_wifi_get_ssid(void)
{
    return s_ssid;
}

uint8_t app_wifi_get_sta_count(void)
{
    return s_sta_count;
}

esp_netif_t *app_wifi_get_netif(void)
{
    return s_ap_netif;
}

#endif  /* SOC_WIFI_SUPPORTED */
