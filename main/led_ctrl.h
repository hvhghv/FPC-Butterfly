/*
 * 蝴蝶板灯珠控制模块
 *
 * 硬件: 4 颗共阴极 RGB 灯珠，每颗 3 路 PWM (R/G/B)，共 12 路。
 *
 *   | 灯珠 | 位置坐标   | 红    | 绿    | 蓝    | 公共阴极 |
 *   |------|------------|-------|-------|-------|----------|
 *   | D1   | (17, 22)mm | IO4   | IO5   | IO6   | GND      |
 *   | D2   | (22, 39)mm | IO0   | IO1   | IO7   | GND      |
 *   | D3   | (53, 22)mm | IO18  | IO19  | IO20  | GND      |
 *   | D4   | (48, 39)mm | IO21  | IO22  | IO23  | GND      |
 *
 * 驱动方式: 双硬件 PWM 外设 (LEDC + MCPWM)，全部 12 路零 CPU 占用
 *
 *   ESP32-C6 的 LEDC 外设只有 6 个通道
 *   (soc_caps.h: SOC_LEDC_CHANNEL_NUM = 6)，
 *   但 MCPWM 外设可再提供 6 路独立 PWM
 *   (mcpwm_ll.h: 3 操作器 x 2 比较器/生成器 = 6)，
 *   两者合计正好 12 路，因此采用:
 *
 *     前 6 路 (D1 全部 + D2 全部) -> LEDC 硬件 PWM
 *                                   12 位分辨率，默认 5 kHz
 *     后 6 路 (D3 全部 + D4 全部) -> MCPWM 硬件 PWM
 *                                   共用 1 个定时器，每路独立比较器
 *
 * 两路都是纯硬件 PWM，运行期 CPU 占用为 0，无抖动。
 * 共阴极接法 => 占空比越大越亮，占空比 0 为熄灭。
 *
 * 线程模型:
 *   - 亮度值受互斥锁保护，任何任务/HTTP 处理函数都可安全调用
 *   - 一个 20 ms 周期的效果任务负责计算目标颜色并写入占空比
 *   - 无中断参与调光，效果平滑度只受刷新周期限制
 *
 * 控制粒度:
 *   每颗灯珠都有**独立**的颜色、亮度、效果、效果周期与开关，
 *   互不影响 (例如 D1 呼吸、D2 闪烁、D3 彩虹、D4 静态可同时运行)。
 *   全局接口 (led_ctrl_set_effect / set_brightness / ...) 保留为
 *   "一次性同步到所有灯珠" 的便捷封装，便于一键统一。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 硬件常量
 * ========================================================================== */

/** 灯珠数量 */
#define LED_CTRL_COUNT          4

/** 每颗灯珠的通道数 (R/G/B) */
#define LED_CTRL_CH_PER_LED     3

/** PWM 总通道数 */
#define LED_CTRL_CH_TOTAL       (LED_CTRL_COUNT * LED_CTRL_CH_PER_LED)

/** LEDC 硬件 PWM 通道数 (ESP32-C6 仅 6 个) */
#define LED_CTRL_LEDC_CH_NUM    6

/** MCPWM 硬件 PWM 通道数 (3 操作器 x 2 = 6) */
#define LED_CTRL_MCPWM_CH_NUM   (LED_CTRL_CH_TOTAL - LED_CTRL_LEDC_CH_NUM)

/** LEDC 硬件 PWM 分辨率 (位) */
#define LED_CTRL_RES_BITS       12

/** LEDC 硬件 PWM 满量程值 (2^12 - 1) */
#define LED_CTRL_DUTY_MAX       4095

/** MCPWM 计数器分辨率 (Hz) — 1 MHz 时每 tick = 1 us */
#define LED_CTRL_MCPWM_RES_HZ   1000000

/** LEDC 默认 PWM 频率 (Hz) */
#define LED_CTRL_DEFAULT_FREQ   5000

/** 灯珠名称 (索引 0-3 分别对应 D1-D4) */
extern const char *const led_ctrl_names[LED_CTRL_COUNT];

/** 灯珠在板上的位置 (mm)，索引 0-3 分别对应 D1-D4 */
extern const uint8_t led_ctrl_pos_x[LED_CTRL_COUNT];
extern const uint8_t led_ctrl_pos_y[LED_CTRL_COUNT];

/** 每颗灯珠 R/G/B 对应的 GPIO，索引 0-3 分别对应 D1-D4 */
extern const int led_ctrl_gpio[LED_CTRL_COUNT][LED_CTRL_CH_PER_LED];

/* ============================================================================
 * 动态效果
 * ========================================================================== */

typedef enum {
    LED_EFFECT_NONE = 0,    /*!< 静态显示当前颜色 */
    LED_EFFECT_BREATH,      /*!< 呼吸 (整体亮度平滑起伏) */
    LED_EFFECT_BLINK,       /*!< 闪烁 (亮/灭交替) */
    LED_EFFECT_RAINBOW,     /*!< 彩虹 (色相循环流动) */
    LED_EFFECT_CHASE,       /*!< 流水 (逐颗灯珠点亮) */
    LED_EFFECT_MAX,
} led_effect_t;

/** 效果名称 (与 led_effect_t 一一对应) */
extern const char *const led_effect_names[LED_EFFECT_MAX];

/* ============================================================================
 * 效果序列 (多效果组合循环)
 * ========================================================================== */

/**
 * 单颗灯珠最多可编排的步骤数
 *
 * 每步占 12 字节，4 颗灯珠全满也只有 384 字节，内存压力可忽略。
 */
#define LED_SEQ_MAX_STEPS       8

/**
 * @brief 效果序列中的一个步骤
 *
 * 把若干步骤按时长依次播放，播完最后一步后回到第一步，无限循环。
 * 例如「1 秒静态 -> 1 秒闪烁 -> 1 秒呼吸」就是三步的序列。
 *
 * 每步可**独立指定颜色与亮度**，实现「暗红呼吸 -> 亮绿闪烁 -> 蓝色彩虹」
 * 这类多彩循环。use_color 为 false 时沿用灯珠的全局颜色与亮度。
 */
typedef struct {
    uint8_t  effect;        /*!< 该步骤的效果 led_effect_t */
    uint32_t duration_ms;   /*!< 该步骤持续时长 (毫秒)，10 - 600000 */
    uint32_t period_ms;     /*!< 该步骤内效果的周期 (毫秒)，0 = 用 duration */
    /*
     * 该步骤的颜色与亮度。
     *
     * use_color 为 true 时用 rgb/brightness 覆盖灯珠全局设置；
     * 为 false 时沿用灯珠全局颜色与亮度 (向后兼容旧配置)。
     */
    uint8_t  r;             /*!< 红色分量 (0-255) */
    uint8_t  g;             /*!< 绿色分量 (0-255) */
    uint8_t  b;             /*!< 蓝色分量 (0-255) */
    uint8_t  brightness;    /*!< 该步骤亮度缩放 (0-255) */
    bool     use_color;     /*!< 是否使用本步骤颜色与亮度 */
} led_step_t;

/**
 * @brief 设置单颗灯珠的效果序列
 *
 * 序列非空时，该灯珠的效果由序列驱动 (逐步骤播放并循环)；
 * 序列为空 (step_count = 0) 时，回退到 led_ctrl_set_led_effect() 设置的单效果。
 *
 * 设置后立即从第一步开始播放。
 *
 * @param idx        灯珠索引 (0-3)
 * @param steps      步骤数组，step_count 为 0 时可为 NULL
 * @param step_count 步骤数 (0 - LED_SEQ_MAX_STEPS)
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引/步骤数/步骤内容非法
 */
esp_err_t led_ctrl_set_led_sequence(uint8_t idx, const led_step_t *steps,
                                    uint8_t step_count);

/**
 * @brief 读取单颗灯珠的效果序列
 *
 * @param idx        灯珠索引 (0-3)
 * @param[out] steps 输出缓冲区，可为 NULL (只查数量)
 * @param[in,out] step_count 传入缓冲区容量，返回实际步骤数
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引越界
 *         ESP_ERR_INVALID_SIZE 缓冲区不足
 */
esp_err_t led_ctrl_get_led_sequence(uint8_t idx, led_step_t *steps,
                                    uint8_t *step_count);

/**
 * @brief 清空单颗灯珠的效果序列
 *
 * 清空后该灯珠回退到单效果 (effect/period 字段)。
 *
 * @param idx 灯珠索引 (0-3)
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引越界
 */
esp_err_t led_ctrl_clear_led_sequence(uint8_t idx);

/**
 * @brief 设置全局效果序列 (同步到所有灯珠)
 *
 * @param steps      步骤数组，step_count 为 0 时可为 NULL
 * @param step_count 步骤数 (0 - LED_SEQ_MAX_STEPS)
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 参数非法
 */
esp_err_t led_ctrl_set_sequence(const led_step_t *steps, uint8_t step_count);

/**
 * @brief 清空所有灯珠的效果序列
 *
 * @return ESP_OK 成功
 */
esp_err_t led_ctrl_clear_sequence(void);

/* ============================================================================
 * 接口
 * ========================================================================== */

/**
 * @brief 初始化灯珠控制模块
 *
 * 配置 12 路 LEDC 通道并创建效果刷新任务。调用后所有灯珠熄灭。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE 已初始化
 *         其他 LEDC 配置失败
 */
esp_err_t led_ctrl_init(void);

/**
 * @brief 停止效果刷新任务并关闭所有 PWM 输出
 *
 * @return ESP_OK 成功
 */
esp_err_t led_ctrl_deinit(void);

/**
 * @brief 设置单颗灯珠的颜色
 *
 * @param idx 灯珠索引 (0-3)
 * @param r   红色亮度 (0-255)
 * @param g   绿色亮度 (0-255)
 * @param b   蓝色亮度 (0-255)
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引越界
 */
esp_err_t led_ctrl_set_rgb(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 读取单颗灯珠当前颜色
 *
 * @param idx 灯珠索引 (0-3)
 * @param[out] r/g/b 输出亮度，可为 NULL
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引越界
 */
esp_err_t led_ctrl_get_rgb(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b);

/**
 * @brief 所有灯珠设置为同一颜色
 *
 * @param r/g/b 亮度 (0-255)
 * @return ESP_OK 成功
 */
esp_err_t led_ctrl_set_all(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 关闭所有灯珠 (亮度清零，保留当前颜色设置)
 *
 * @return ESP_OK 成功
 */
esp_err_t led_ctrl_all_off(void);

/**
 * @brief 设置全局亮度缩放系数 (同步到所有灯珠)
 *
 * 实际输出 = 设定颜色 * 该灯珠亮度 / 255。
 * 这是便捷封装，等价于对每颗灯珠调用 led_ctrl_set_led_brightness()。
 *
 * @param brightness 0-255
 * @return ESP_OK 成功
 */
esp_err_t led_ctrl_set_brightness(uint8_t brightness);

/**
 * @brief 读取全局亮度缩放系数
 *
 * 返回所有灯珠亮度的最大值，用于 UI 显示"整体亮度"。
 * 若各灯珠亮度不同，应改用 led_ctrl_get_led_brightness()。
 *
 * @return 0-255
 */
uint8_t led_ctrl_get_brightness(void);

/**
 * @brief 设置单颗灯珠的亮度缩放系数
 *
 * @param idx        灯珠索引 (0-3)
 * @param brightness 0-255
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引越界
 */
esp_err_t led_ctrl_set_led_brightness(uint8_t idx, uint8_t brightness);

/**
 * @brief 读取单颗灯珠的亮度缩放系数
 *
 * @param idx 灯珠索引 (0-3)
 * @return 0-255，索引越界返回 0
 */
uint8_t led_ctrl_get_led_brightness(uint8_t idx);

/**
 * @brief 设置全局动态效果 (同步到所有灯珠)
 *
 * @param effect 效果类型
 * @param period_ms 效果周期 (毫秒)，0 表示保持各自当前周期
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 效果类型非法
 */
esp_err_t led_ctrl_set_effect(led_effect_t effect, uint32_t period_ms);

/**
 * @brief 读取当前效果
 *
 * 若各灯珠效果不同，返回第一颗灯珠的效果。
 * 需要逐颗状态请使用 led_ctrl_get_led_effect()。
 *
 * @param[out] effect    输出效果类型，可为 NULL
 * @param[out] period_ms 输出效果周期，可为 NULL
 * @return ESP_OK 成功
 */
esp_err_t led_ctrl_get_effect(led_effect_t *effect, uint32_t *period_ms);

/**
 * @brief 设置单颗灯珠的动态效果
 *
 * 每颗灯珠的效果相位独立推进，因此不同周期互不干扰。
 *
 * @param idx       灯珠索引 (0-3)
 * @param effect    效果类型
 * @param period_ms 效果周期 (毫秒)，0 表示保持当前周期
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引或效果类型非法
 */
esp_err_t led_ctrl_set_led_effect(uint8_t idx, led_effect_t effect,
                                  uint32_t period_ms);

/**
 * @brief 读取单颗灯珠的动态效果
 *
 * @param idx 灯珠索引 (0-3)
 * @param[out] effect    输出效果类型，可为 NULL
 * @param[out] period_ms 输出效果周期，可为 NULL
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引越界
 */
esp_err_t led_ctrl_get_led_effect(uint8_t idx, led_effect_t *effect,
                                  uint32_t *period_ms);

/**
 * @brief 启用/禁用单颗灯珠
 *
 * 禁用后该灯珠输出全灭，但颜色/效果设置保留，重新启用即恢复。
 *
 * @param idx     灯珠索引 (0-3)
 * @param enabled true 启用，false 禁用
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 索引越界
 */
esp_err_t led_ctrl_set_led_enabled(uint8_t idx, bool enabled);

/**
 * @brief 查询单颗灯珠是否启用
 *
 * @param idx 灯珠索引 (0-3)
 * @return true 启用；索引越界返回 false
 */
bool led_ctrl_get_led_enabled(uint8_t idx);

/**
 * @brief 设置单颗灯珠的完整状态 (一次调用配置全部高级参数)
 *
 * 未提供的字段用 NULL 表示"保持当前值"，便于只改其中一项。
 *
 * @param idx        灯珠索引 (0-3)
 * @param r/g/b      颜色，可为 NULL
 * @param brightness 亮度，可为 NULL
 * @param effect     效果，可为 NULL
 * @param period_ms  效果周期，可为 NULL
 * @param enabled    开关，可为 NULL
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 参数非法
 */
esp_err_t led_ctrl_set_led_state(uint8_t idx,
                                 const uint8_t *r, const uint8_t *g,
                                 const uint8_t *b,
                                 const uint8_t *brightness,
                                 const led_effect_t *effect,
                                 const uint32_t *period_ms,
                                 const bool *enabled);

/**
 * @brief 设置 PWM 频率
 *
 * LEDC 与 MCPWM 两个外设同步改频。MCPWM 的计数器分辨率固定为
 * LED_CTRL_MCPWM_RES_HZ (1 MHz)，改频时重新计算周期 tick 数。
 *
 * @param freq_hz 频率，范围 100 - 40000
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 频率越界
 */
esp_err_t led_ctrl_set_freq(uint32_t freq_hz);

/**
 * @brief 读取当前 PWM 频率
 *
 * @return 频率 (Hz)
 */
uint32_t led_ctrl_get_freq(void);

/**
 * @brief 按名称查找效果类型
 *
 * @param name 效果名 ("none" / "breath" / "blink" / "rainbow" / "chase")
 * @return 效果类型，未找到返回 LED_EFFECT_MAX
 */
led_effect_t led_ctrl_effect_from_name(const char *name);

/**
 * @brief 设置灯珠映射是否反转 (公共阳极接法)
 *
 * 默认 false (共阴极: 占空比越大越亮)。
 * 若实际硬件为公共阳极接 VCC，可置 true 使逻辑保持一致。
 *
 * @param invert true 反转输出
 * @return ESP_OK 成功
 */
esp_err_t led_ctrl_set_invert(bool invert);

/**
 * @brief 查询输出是否反转
 *
 * @return true 表示反转
 */
bool led_ctrl_get_invert(void);

/**
 * @brief 生成所有灯珠状态的 JSON 字符串
 *
 * 每颗灯珠包含独立的高级参数 (brightness/effect/period/enabled)
 * 以及效果序列 (steps):
 *   {"count":4,"leds":[
 *      {"id":1,"name":"D1 左上翅膀","pos":"(17, 22) mm",
 *       "gpio":{"r":4,"g":5,"b":6},"r":255,"g":0,"b":0,
 *       "brightness":255,"effect":"breath","period":2000,"enabled":true,
 *       "steps":[{"effect":"none","duration":1000,"period":0},
 *                 {"effect":"blink","duration":1000,"period":0}]},
 *      ...],
 *    "brightness":255,"effect":"none","period":2000,
 *    "freq":5000,"invert":false,"off":false}
 *
 * 顶层 brightness/effect/period 为"汇总值"(取首颗)，
 * 用于旧前端与一键同步 UI；逐颗真实值在 leds[] 内。
 *
 * @param[out] buf     输出缓冲区
 * @param[in]  buf_len 缓冲区长度，建议 >= 3000
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_SIZE 缓冲区不足
 */
esp_err_t led_ctrl_to_json(char *buf, size_t buf_len);

/* ============================================================================
 * 配置持久化 (NVS)
 *
 * 把灯珠状态 (颜色/亮度/效果/周期/序列/频率/反转/熄灭) 保存到 NVS，
 * 上电时由 led_ctrl_load() 自动恢复，实现"断电重启后保持上次配置"。
 *
 * 保存是**手动**的: 只有用户显式调用 led_ctrl_save() (对应 config.save
 * 命令 / 前端「保存当前配置」按钮) 才落盘；其他设置函数只改运行状态。
 *
 * 典型用法:
 *   led_ctrl_init();
 *   led_ctrl_load();          // 有保存记录则恢复，否则用默认值
 *   ... 用户修改后 ...
 *   led_ctrl_save();          // 用户点「保存」时才落盘
 * ========================================================================== */

/**
 * @brief 保存当前灯珠状态到 NVS
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE 模块未初始化
 *         ESP_ERR_NO_MEM 内存不足
 *         其他 NVS 错误
 */
esp_err_t led_ctrl_save(void);

/**
 * @brief 从 NVS 加载灯珠状态并立即生效
 *
 * 应在 led_ctrl_init() 之后调用。无有效记录时保持默认状态。
 *
 * @return ESP_OK 成功恢复
 *         ESP_ERR_NOT_FOUND 无有效保存记录 (使用默认值)
 *         ESP_ERR_INVALID_STATE 模块未初始化
 *         其他 NVS 错误
 */
esp_err_t led_ctrl_load(void);

/**
 * @brief 清除已保存的灯珠配置
 *
 * 删除 NVS 记录但不改变当前运行状态；下次上电使用默认值。
 *
 * @return ESP_OK 成功 (含本就无记录)
 */
esp_err_t led_ctrl_clear_saved(void);

#ifdef __cplusplus
}
#endif
