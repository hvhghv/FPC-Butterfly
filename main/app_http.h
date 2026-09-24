/*
 * 蝴蝶板灯控应用 — HTTP 服务模块
 *
 * 通过 WiFi AP 访问的 HTTP 服务，提供:
 *
 *   GET  /                Web 控制界面 (HTML)
 *   GET  /api/status      灯珠状态与系统信息 (JSON)
 *
 * 逐颗控制 (每颗灯珠独立颜色/亮度/效果/周期/开关):
 *   POST /api/led         设置单颗灯珠完整状态 (JSON, 字段可选)
 *   POST /api/led/effect  设置单颗灯珠效果 (JSON)
 *   POST /api/led/brightness 设置单颗灯珠亮度 (JSON)
 *   POST /api/led/enable  启用/禁用单颗灯珠 (JSON)
 *
 * 全局控制 (一次性同步到所有灯珠):
 *   POST /api/all         设置全部灯珠颜色 (JSON)
 *   POST /api/brightness  设置全局亮度 (JSON)
 *   POST /api/effect      设置全局效果 (JSON)
 *   POST /api/off         全部熄灭
 *   POST /api/freq        设置 PWM 频率 (JSON)
 *   POST /api/reboot      重启设备
 *   POST /api/upgrade     重启进入 IAP 下载模式 (用于升级固件)
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 HTTP 服务
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE 已启动
 *         其他 httpd_start 失败
 */
esp_err_t app_http_start(void);

/**
 * @brief 停止 HTTP 服务
 *
 * @return ESP_OK 成功
 */
esp_err_t app_http_stop(void);

/**
 * @brief 查询 HTTP 服务是否运行中
 *
 * @return true 运行中
 */
bool app_http_is_running(void);

#ifdef __cplusplus
}
#endif
