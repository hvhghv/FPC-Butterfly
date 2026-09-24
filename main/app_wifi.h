/*
 * 蝴蝶板灯控应用 — WiFi 模块
 *
 * 支持 AP / STA / APSTA 三种模式，参数优先从 NVS 读取
 * (可通过 Web 界面或 REST API 修改并持久化)，其次从 IAP 配置区读取，
 * 最后退回本模块内置的默认值。
 *
 * 参数优先级: NVS 用户配置 > IAP 启动参数 > 内置默认值
 *
 * APSTA 模式下设备既开热点又连路由器:
 *   - 热点保留为**兜底通道**，路由器连不上时仍可访问控制页面
 *   - 连上路由器后可通过路由器分配的 IP 访问，便于接入局域网
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 默认 AP SSID (NVS / IAP 配置均不可用时使用) */
#define APP_WIFI_DEFAULT_SSID       "ESP-LED"

/** 默认 AP 密码 (NVS / IAP 配置均不可用时使用) */
#define APP_WIFI_DEFAULT_PASSWORD   "12345678"

/** 默认 AP 信道 */
#define APP_WIFI_DEFAULT_CHANNEL    1

/** 默认最大客户端数 */
#define APP_WIFI_DEFAULT_MAX_CONN   4

/** SSID 缓冲区长度 (802.11 上限 32 字符 + 结尾 NUL) */
#define APP_WIFI_SSID_MAX           33

/** 密码缓冲区长度 (WPA2 上限 63 字符 + 结尾 NUL) */
#define APP_WIFI_PASS_MAX           65

/** IP 字符串缓冲区长度 ("255.255.255.255" + NUL) */
#define APP_WIFI_IP_MAX             16

/** 扫描结果最大条数 */
#define APP_WIFI_SCAN_MAX           16

/**
 * @brief WiFi 工作模式
 */
typedef enum {
    APP_WIFI_MODE_AP = 0,   /*!< 仅热点 (默认，最安全) */
    APP_WIFI_MODE_STA,      /*!< 仅连路由器 (⚠️ 连不上会失联) */
    APP_WIFI_MODE_APSTA,    /*!< 热点 + 连路由器 (推荐) */
} app_wifi_mode_t;

/**
 * @brief AP / STA 运行参数
 *
 * 通过 app_wifi_set_cfg() 修改后会写入 NVS 并立即生效。
 */
typedef struct {
    /* --- 工作模式 --- */
    uint8_t  mode;                      /*!< app_wifi_mode_t */

    /* --- AP (热点) 参数 --- */
    char     ssid[APP_WIFI_SSID_MAX];   /*!< 热点名称 (1-32 字符) */
    char     password[APP_WIFI_PASS_MAX]; /*!< 密码 (8-63 字符；空串 = 开放网络) */
    uint8_t  channel;                   /*!< 信道 1-13 */
    uint8_t  max_conn;                  /*!< 最大客户端数 1-10 */
    bool     hidden;                    /*!< 是否隐藏 SSID */
    char     ip[APP_WIFI_IP_MAX];       /*!< AP 的 IPv4 地址，如 "192.168.4.1" */

    /* --- STA (连路由器) 参数 --- */
    char     sta_ssid[APP_WIFI_SSID_MAX]; /*!< 要连接的路由器 SSID */
    char     sta_password[APP_WIFI_PASS_MAX]; /*!< 路由器密码 (空 = 开放网络) */
} app_wifi_cfg_t;

/**
 * @brief STA 连接状态
 */
typedef struct {
    bool     enabled;                   /*!< 是否启用了 STA */
    bool     connected;                 /*!< 是否已连上路由器 */
    char     ssid[APP_WIFI_SSID_MAX];   /*!< 已连接(或正在连接)的 SSID */
    char     ip[APP_WIFI_IP_MAX];       /*!< 从路由器获取的 IP，未连接时为 "0.0.0.0" */
    char     gw[APP_WIFI_IP_MAX];       /*!< 网关地址 */
    int8_t   rssi;                      /*!< 信号强度 dBm，未连接时为 0 */
} app_wifi_sta_status_t;

/**
 * @brief 扫描到的单个 AP
 */
typedef struct {
    char     ssid[APP_WIFI_SSID_MAX];   /*!< SSID */
    int8_t   rssi;                      /*!< 信号强度 dBm */
    uint8_t  channel;                   /*!< 信道 */
    uint8_t  authmode;                  /*!< wifi_auth_mode_t */
} app_wifi_ap_info_t;

/**
 * @brief 读取当前 WiFi 配置
 *
 * 返回的是本模块内存中的当前生效值 (启动时已从 NVS 加载)。
 *
 * @param[out] cfg 输出配置
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG cfg 为 NULL
 */
esp_err_t app_wifi_get_cfg(app_wifi_cfg_t *cfg);

/**
 * @brief 修改 WiFi 配置 (写入 NVS 并立即生效)
 *
 * 校验规则:
 *   - mode: 必须是合法的 app_wifi_mode_t
 *   - AP 参数仅在 mode 含 AP 时校验:
 *       ssid 1-32 字符；password 空串(开放) 或 8-63 字符；
 *       channel 1-13；max_conn 1-10；ip 合法且必须以 .1 结尾
 *   - STA 参数仅在 mode 含 STA 时校验:
 *       sta_ssid 1-32 字符；sta_password 空串(开放) 或 8-63 字符
 *
 * 注意: 修改 SSID / 密码 / 信道 / 模式 会导致已连接的客户端断开，
 *       调用方应先把 HTTP 响应发回浏览器再调用本函数。
 *
 * @param[in] cfg 新配置
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 参数非法
 *         ESP_ERR_INVALID_STATE WiFi 未启动
 *         其他 NVS 写入失败
 */
esp_err_t app_wifi_set_cfg(const app_wifi_cfg_t *cfg);

/**
 * @brief 恢复出厂默认配置 (清除 NVS 中的用户配置)
 *
 * 不会立即重启 WiFi，需调用方自行重启设备或再次调用
 * app_wifi_set_cfg() 应用默认值。
 *
 * @return ESP_OK 成功
 */
esp_err_t app_wifi_reset_cfg(void);

/**
 * @brief 读取 STA 连接状态
 *
 * @param[out] st 输出状态
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG st 为 NULL
 */
esp_err_t app_wifi_get_sta_status(app_wifi_sta_status_t *st);

/**
 * @brief 主动触发 STA 重连
 *
 * 用于「连不上路由器时手动重试」。会先断开再重新连接。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE STA 未启用或 WiFi 未启动
 */
esp_err_t app_wifi_sta_reconnect(void);

/**
 * @brief 扫描周边 WiFi 热点
 *
 * 阻塞执行，约需 2-4 秒。扫描期间 STA 会短暂断开。
 *
 * @param[out] list      输出数组
 * @param[in]  max       数组容量
 * @param[out] found     实际找到的数量
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 参数非法
 *         ESP_ERR_INVALID_STATE WiFi 未启动
 */
esp_err_t app_wifi_scan(app_wifi_ap_info_t *list, uint8_t max, uint8_t *found);

/**
 * @brief 启动 WiFi
 *
 * 按配置的 mode 启动 AP / STA / APSTA。
 * 阻塞直到 AP 就绪或超时 (10 秒)；STA 连接在后台异步进行。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_NOT_SUPPORTED 芯片无 WiFi 硬件
 *         其他 初始化失败
 */
esp_err_t app_wifi_start(void);

/**
 * @brief 启动 SoftAP (非阻塞)
 *
 * 立即返回，AP 在后台完成启动。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_NOT_SUPPORTED 芯片无 WiFi 硬件
 */
esp_err_t app_wifi_start_nonblocking(void);

/**
 * @brief 停止 SoftAP
 *
 * @return ESP_OK 成功
 */
esp_err_t app_wifi_stop(void);

/**
 * @brief 获取 AP 的 IP 地址字符串
 *
 * @return 静态字符串，如 "192.168.4.1"；未启动时返回 "0.0.0.0"
 */
const char *app_wifi_get_ip(void);

/**
 * @brief 获取 AP 的 SSID
 *
 * @return 静态字符串
 */
const char *app_wifi_get_ssid(void);

/**
 * @brief 获取当前接入 AP 的客户端数量
 *
 * @return 客户端数量
 */
uint8_t app_wifi_get_sta_count(void);

/**
 * @brief 获取 AP 网卡句柄
 *
 * @return 网卡句柄，未启动时返回 NULL
 */
esp_netif_t *app_wifi_get_netif(void);

#ifdef __cplusplus
}
#endif
