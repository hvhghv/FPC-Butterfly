/*
 * 蝴蝶板灯控应用 — BLE 蓝牙控制模块
 *
 * 通过 BLE GATT 提供与 HTTP 等价的控制能力，供浏览器
 * (Web Bluetooth API) 或手机 App 直接连接控制。
 *
 * 【GATT 结构】
 *
 *   Service  (自定义 128 位 UUID)
 *     ├─ RX 特征 (Write)     浏览器写入命令 JSON
 *     ├─ TX 特征 (Notify)    设备回传响应 JSON
 *     └─ MTU 特征 (Read)     告知客户端可用的最大分片长度
 *
 * 【分片协议】
 *
 * BLE 单包有效载荷受 MTU 限制 (默认 20 字节，协商后最大 244 字节)。
 * 命令 JSON 可能超过该长度，因此采用简单的长度前缀分片:
 *
 *   首片: [总长度 2 字节小端][数据...]
 *   后续: [数据...]
 *
 * 接收端按总长度收齐后交给命令层；发送端把响应按 MTU 切片后
 * 逐包 notify。
 *
 * 【与 WiFi 共存】
 *
 * BLE 与 WiFi 共用 2.4 GHz 射频，必须开启软件共存
 * (CONFIG_SW_COEXIST_ENABLE)，否则会出现连接不稳或吞吐骤降。
 *
 * 【内存】
 *
 * 使用 NimBLE 协议栈 (比 Bluedroid 省约 30-50KB DRAM)。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 设备在蓝牙广播中显示的名称前缀 */
#define APP_BLE_DEVICE_NAME     "Butterfly-LED"

/** 配对码长度 (6 位数字，符合 BLE 规范) */
#define APP_BLE_PIN_LEN         6

/**
 * @brief 设置 BLE 配对码
 *
 * 启用配对后，客户端连接时需要输入该 6 位数字码。
 * 设置后立即生效 (下次配对时使用)，并写入 NVS 持久保存。
 *
 * @param pin 6 位数字字符串，如 "123456"；传 NULL 或空串表示禁用配对
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 格式非法 (必须是 6 位数字)
 *         ESP_ERR_INVALID_STATE BLE 未启动
 */
esp_err_t app_ble_set_pin(const char *pin);

/**
 * @brief 查询是否启用了配对
 *
 * @return true 已启用 (需要配对码)
 */
bool app_ble_pairing_enabled(void);

/**
 * @brief 查询当前配对码
 *
 * 仅在 app_ble_pairing_enabled() 为 true 时有意义。
 *
 * @return 6 位数字字符串；未启用时返回空串
 */
const char *app_ble_get_pin(void);

/**
 * @brief 查询当前连接是否已加密 (已通过配对)
 *
 * 用于权限判断: 配对码本身是敏感信息，只应允许**已配对**的
 * 客户端读取，否则配对机制形同虚设。
 *
 * @return true 已连接且链路已加密
 */
bool app_ble_is_encrypted(void);

/**
 * @brief 启动 BLE 服务
 *
 * 初始化 NimBLE 协议栈、注册 GATT 服务并开始广播。
 * 若芯片不支持 BLE 则返回 ESP_ERR_NOT_SUPPORTED。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE 已启动
 *         ESP_ERR_NOT_SUPPORTED 芯片无 BLE 硬件
 *         其他 初始化失败
 */
esp_err_t app_ble_start(void);

/**
 * @brief 停止 BLE 服务
 *
 * @return ESP_OK 成功
 */
esp_err_t app_ble_stop(void);

/**
 * @brief 查询 BLE 是否已启动
 *
 * @return true 已启动
 */
bool app_ble_is_running(void);

/**
 * @brief 查询当前是否有客户端已连接
 *
 * @return true 已连接
 */
bool app_ble_is_connected(void);

/**
 * @brief 主动断开当前连接
 *
 * 用于「断开」按钮，或在切换配置后强制客户端重连。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE 无连接
 */
esp_err_t app_ble_disconnect(void);

#ifdef __cplusplus
}
#endif
