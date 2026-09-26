/*
 * 蝴蝶板灯控应用 — 命令层 (传输无关)
 *
 * 本模块把「控制命令」从传输层 (HTTP / BLE) 中解耦出来:
 *
 *     传输层                     命令层                业务层
 *   ┌──────────┐            ┌─────────────┐       ┌──────────┐
 *   │ HTTP     │──JSON────▶ │             │─────▶ │ led_ctrl │
 *   │ BLE      │◀──JSON──── │ app_cmd     │ ◀──── │ app_wifi │
 *   └──────────┘            └─────────────┘       └──────────┘
 *
 * 这样新增一种传输方式 (如 BLE) 时，只需把收到的 JSON 交给
 * app_cmd_execute()，无需重复实现任何业务逻辑。
 *
 * 命令格式与 HTTP API 保持一致，便于前端复用同一套调用代码:
 *
 *   {"cmd":"led.set",  "id":0,"r":255,"g":0,"b":0}
 *   {"cmd":"led.effect","id":0,"name":"breath","period":2000}
 *   {"cmd":"status"}
 *   {"cmd":"battery.get"}                     读取电池信息 (IP5108)
 *   {"cmd":"wifi.get"}
 *   {"cmd":"config.export"}                    导出配置 (不含密码)
 *   {"cmd":"config.export","secrets":1}        导出配置 (含密码明文)
 *   {"cmd":"config.import", ...}               导入配置
 *   {"cmd":"config.save"}                      保存当前配置到 NVS (手动保存)
 *   {"cmd":"config.reset"}                     清除已保存配置 (下次上电用默认值)
 *
 * 响应统一为:
 *   {"ok":true, ...}   或   {"ok":false,"error":"..."}
 *
 * 【配置持久化】
 *   灯珠状态的保存是**手动**的: 只有 config.save 命令 (前端「保存当前
 *   配置」按钮) 才会写入 NVS。其他修改类命令 (led.set / led.effect /
 *   led.brightness / led.enable / led.sequence / all / brightness /
 *   effect / off / freq) 仅改变运行状态，不落盘 —— 这样可避免拖动
 *   滑条时频繁写 flash。
 *   保存后，断电重启时由 main.c 启动流程中的 led_ctrl_load() 自动恢复。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 命令响应缓冲区建议长度。
 *
 * status 与 config.export 响应最大 —— 后者含 4 颗灯珠 × 8 步序列
 * (每步带独立颜色) 加 WiFi 配置，约 4KB，取 8KB 留足余量。
 */
#define APP_CMD_RESP_MAX    8192

/**
 * @brief 命令来源通道
 *
 * 部分命令涉及敏感信息 (如蓝牙配对码)，需要按来源做权限判断:
 *   - HTTP: 已通过 WiFi 密码保护，视为可信
 *   - BLE:  需链路已加密 (已配对) 才允许读取
 */
typedef enum {
    APP_CMD_SRC_HTTP = 0,   /*!< 来自 HTTP 服务 */
    APP_CMD_SRC_BLE,        /*!< 来自 BLE GATT */
    APP_CMD_SRC_LOCAL,      /*!< 本地调用 (串口/内部)，视为可信 */
} app_cmd_src_t;

/**
 * @brief 执行一条命令 (带来源信息)
 *
 * 解析 JSON 中的 "cmd" 字段并分发到对应处理函数，把响应 JSON
 * 写入 out_buf。无论成功失败都会写入合法 JSON (失败时 ok=false)。
 *
 * 本函数线程安全 (内部各业务模块自带锁)，可从 HTTP 任务或 BLE
 * 回调任务调用。
 *
 * @param[in]  json    请求 JSON 文本 (以 '\0' 结尾)
 * @param[out] out_buf 响应缓冲区
 * @param[in]  out_len 响应缓冲区长度
 * @param[in]  src     命令来源通道 (用于权限判断)
 * @return ESP_OK 命令已执行 (响应可能是 ok=false)
 *         ESP_ERR_INVALID_ARG 参数为 NULL
 *         ESP_ERR_INVALID_SIZE 响应缓冲区不足
 */
esp_err_t app_cmd_execute_src(const char *json, char *out_buf, size_t out_len,
                              app_cmd_src_t src);

/**
 * @brief 执行一条命令 (来源视为本地，拥有全部权限)
 *
 * 等价于 app_cmd_execute_src(..., APP_CMD_SRC_LOCAL)。
 */
esp_err_t app_cmd_execute(const char *json, char *out_buf, size_t out_len);

/**
 * @brief 从 JSON 文本中提取整数
 *
 * 极简解析器，只处理本项目用到的扁平 JSON:
 *   {"r":255,"g":0,"b":0}
 *
 * 键名匹配使用 "\"key\"" 精确查找，避免 "r" 误匹配 "brightness"。
 *
 * @param[in]  json JSON 文本
 * @param[in]  key  键名 (不含引号)
 * @param[out] out  输出值
 * @return true 找到并解析成功
 */
bool app_cmd_get_int(const char *json, const char *key, long *out);

/**
 * @brief 从 JSON 文本中提取字符串值
 *
 * @param[in]  json    JSON 文本
 * @param[in]  key     键名 (不含引号)
 * @param[out] out     输出缓冲区
 * @param[in]  out_len 缓冲区长度
 * @return true 找到并解析成功
 */
bool app_cmd_get_str(const char *json, const char *key,
                     char *out, size_t out_len);

/**
 * @brief 把 long 值收敛到 [lo, hi]
 */
long app_cmd_clamp(long v, long lo, long hi);

#ifdef __cplusplus
}
#endif
