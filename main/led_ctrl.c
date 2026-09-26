/*
 * 蝴蝶板灯珠控制模块 — 实现
 *
 * 12 路 PWM (4 颗 RGB 灯珠 x 3 色) + 动态效果引擎。
 *
 * 【为什么用双硬件 PWM 外设】
 *
 * ESP32-C6 的单个 PWM 外设都不够 12 路，但两个外设合计正好够:
 *
 *   LEDC   : soc_caps.h  SOC_LEDC_CHANNEL_NUM = 6
 *   MCPWM  : mcpwm_ll.h  MCPWM_LL_OPERATORS_PER_GROUP      = 3
 *                        MCPWM_LL_COMPARATORS_PER_OPERATOR = 2
 *                        MCPWM_LL_GENERATORS_PER_OPERATOR  = 2
 *                        => 3 x 2 = 6 路 (每路独立比较器 + 生成器)
 *   合计   : 6 + 6 = 12 路 ✅
 *
 * 两者都是纯硬件 PWM，运行期 CPU 占用为 0，无抖动。
 *
 * 通道分配:
 *
 *   序号  灯珠  颜色  GPIO   驱动方式
 *   ----  ----  ----  ----   ----------------
 *     0    D1     R     4    LEDC 通道 0
 *     1    D1     G     5    LEDC 通道 1
 *     2    D1     B     6    LEDC 通道 2
 *     3    D2     R     0    LEDC 通道 3
 *     4    D2     G     1    LEDC 通道 4
 *     5    D2     B     7    LEDC 通道 5
 *     6    D3     R    18    MCPWM 操作器0/比较器0/生成器0
 *     7    D3     G    19    MCPWM 操作器0/比较器1/生成器1
 *     8    D3     B    20    MCPWM 操作器1/比较器0/生成器0
 *     9    D4     R    21    MCPWM 操作器1/比较器1/生成器1
 *    10    D4     G    22    MCPWM 操作器2/比较器0/生成器0
 *    11    D4     B    23    MCPWM 操作器2/比较器1/生成器1
 *
 * MCPWM 波形生成原理 (每个生成器两条规则):
 *   定时器事件 EMPTY (计数归零) -> 输出 HIGH   (周期开始)
 *   比较器事件 (计数 == duty)   -> 输出 LOW    (占空比到达)
 * 6 路共用 1 个定时器，因此载波相位完全同步。
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"

#include "led_ctrl.h"

static const char *TAG = "led_ctrl";

/* ============================================================================
 * 配置持久化 (NVS)
 *
 * 把灯珠状态 (颜色/亮度/效果/周期/序列/频率/反转/熄灭标志) 存到独立
 * 命名空间 "ledcfg"，上电时由 led_ctrl_load() 自动恢复。
 *
 * 存储格式: 单个 blob 键 "state"，内容为 led_state_t 的紧凑副本。
 *   - 用 blob 而非逐字段键: 结构简单、一次读写、原子性更好
 *   - 版本字段: 将来结构变更时可识别并丢弃旧数据，避免误读
 * ========================================================================== */

/** NVS 命名空间 (灯珠配置) */
#define LED_NVS_NAMESPACE   "ledcfg"

/** NVS 键名 (状态 blob) */
#define LED_NVS_KEY_STATE   "state"

/** 持久化数据版本 (结构变更时递增) */
#define LED_NVS_VERSION     1

/**
 * @brief 持久化记录 (与运行期 led_state_t 解耦)
 *
 * 单独定义而非直接存 led_state_t，是为了:
 *   1. 未来运行期结构变更时不影响已存数据布局
 *   2. 明确版本号，便于兼容处理
 */
typedef struct {
    uint16_t version;                                   /*!< 结构版本 */
    uint16_t reserved;                                  /*!< 对齐填充 */
    uint8_t  rgb[LED_CTRL_COUNT][LED_CTRL_CH_PER_LED];  /*!< 各灯珠颜色 */
    uint8_t  brightness[LED_CTRL_COUNT];                /*!< 各灯珠亮度 */
    uint8_t  effect[LED_CTRL_COUNT];                    /*!< 各灯珠效果 */
    uint32_t period_ms[LED_CTRL_COUNT];                 /*!< 各灯珠效果周期 */
    uint8_t  enabled[LED_CTRL_COUNT];                   /*!< 各灯珠使能 */
    led_step_t steps[LED_CTRL_COUNT][LED_SEQ_MAX_STEPS];/*!< 各灯珠效果序列 */
    uint8_t  step_count[LED_CTRL_COUNT];                /*!< 各灯珠步骤数 */
    uint32_t freq_hz;                                   /*!< PWM 频率 */
    uint8_t  invert;                                    /*!< 输出反转 */
    uint8_t  off;                                       /*!< 熄灭标志 */
    uint8_t  pad[2];                                    /*!< 对齐填充 */
} led_nvs_blob_t;

/* ============================================================================
 * 硬件映射表
 * ========================================================================== */

const char *const led_ctrl_names[LED_CTRL_COUNT] = {
    "D1 左上翅膀",
    "D2 左下翅膀",
    "D3 右上翅膀",
    "D4 右下翅膀",
};

const uint8_t led_ctrl_pos_x[LED_CTRL_COUNT] = { 17, 22, 53, 48 };
const uint8_t led_ctrl_pos_y[LED_CTRL_COUNT] = { 22, 39, 22, 39 };

const int led_ctrl_gpio[LED_CTRL_COUNT][LED_CTRL_CH_PER_LED] = {
    { 4,  5,  6  },   /* D1 左上翅膀 */
    { 0,  1,  7  },   /* D2 左下翅膀 */
    { 18, 19, 20 },   /* D3 右上翅膀 */
    { 21, 22, 23 },   /* D4 右下翅膀 */
};

const char *const led_effect_names[LED_EFFECT_MAX] = {
    "none", "breath", "blink", "rainbow", "chase",
};

/* ============================================================================
 * 内部状态
 * ========================================================================== */

/** 效果刷新周期 (毫秒) — 50 Hz */
#define LED_REFRESH_MS          20

/** 默认效果周期 (毫秒) */
#define LED_DEFAULT_PERIOD_MS   2000

/** 效果任务栈大小 */
#define LED_TASK_STACK          3072

/** 效果任务优先级 */
#define LED_TASK_PRIO           4

/** LEDC 使用的定时器与速度模式 (前 6 路共用) */
#define LED_TIMER_NUM           LEDC_TIMER_0
#define LED_SPEED_MODE          LEDC_LOW_SPEED_MODE

/** 全局 PWM 通道序号 -> (灯珠, 颜色) */
#define LED_CH_OF(idx, color)   ((idx) * LED_CTRL_CH_PER_LED + (color))

/** 颜色通道顺序 */
enum { LED_COLOR_R = 0, LED_COLOR_G = 1, LED_COLOR_B = 2 };

/** MCPWM 每个操作器可挂 2 路 (比较器 + 生成器) */
#define LED_MCPWM_PER_OPER     2

/** MCPWM 需要的操作器数量 */
#define LED_MCPWM_OPER_NUM      (LED_CTRL_MCPWM_CH_NUM / LED_MCPWM_PER_OPER)

/** 互斥锁保护的状态 */
typedef struct {
    uint8_t  rgb[LED_CTRL_COUNT][LED_CTRL_CH_PER_LED]; /*!< 各灯珠设定颜色 (0-255) */
    uint8_t  brightness[LED_CTRL_COUNT]; /*!< 各灯珠亮度缩放 (0-255) */
    uint8_t  effect[LED_CTRL_COUNT];      /*!< 各灯珠效果 led_effect_t */
    uint32_t period_ms[LED_CTRL_COUNT];   /*!< 各灯珠效果周期 */
    bool     enabled[LED_CTRL_COUNT];     /*!< 各灯珠使能 (false = 输出全灭) */
    /*
     * 效果序列: step_count > 0 时覆盖 effect/period_ms 字段，
     * 由序列逐步驱动；step_count == 0 时回退到单效果。
     */
    led_step_t steps[LED_CTRL_COUNT][LED_SEQ_MAX_STEPS];
    uint8_t  step_count[LED_CTRL_COUNT];
    uint32_t freq_hz;           /*!< PWM 频率 (硬件共享，全局) */
    bool     invert;            /*!< 输出电平反转 (公共阳极) */
    bool     off;               /*!< 全部熄灭标志 (保留颜色与效果设置) */
} led_state_t;

static led_state_t        s_state;
static SemaphoreHandle_t  s_lock       = NULL;
static TaskHandle_t       s_task       = NULL;
static bool               s_inited     = false;
static volatile bool      s_task_run   = false;

/**
 * 每颗灯珠的效果相位累加器 (毫秒)
 *
 * 各灯珠周期可能不同，因此相位必须独立推进 —— 若共用单一相位，
 * 周期不同的灯珠会互相拖拽，导致效果节奏错乱。
 * 仅由效果任务访问，无需加锁。
 */
static uint32_t           s_phase_ms[LED_CTRL_COUNT];

/**
 * 每颗灯珠的序列播放进度 (仅效果任务访问，无需加锁)
 *
 *   step_idx      当前播放到第几步
 *   step_elapsed  当前步骤已播放的毫秒数
 */
static uint8_t            s_step_idx[LED_CTRL_COUNT];
static uint32_t           s_step_elapsed[LED_CTRL_COUNT];

/**
 * LEDC 写操作互斥锁
 *
 * ledc_set_duty() / ledc_update_duty() 不是线程安全的，
 * 而效果任务与 HTTP 处理函数都可能调用 write_channel()，
 * 因此用独立锁串行化 LEDC 写入。
 */
static SemaphoreHandle_t  s_ledc_lock  = NULL;

/* ---------------------------------------------------------------------------
 * MCPWM 资源句柄
 *
 * 1 个定时器 + 3 个操作器，每个操作器挂 2 个比较器和 2 个生成器，
 * 共 6 路独立 PWM。
 * ------------------------------------------------------------------------- */
static mcpwm_timer_handle_t s_mcpwm_timer = NULL;
static mcpwm_oper_handle_t  s_mcpwm_oper[LED_MCPWM_OPER_NUM];
static mcpwm_cmpr_handle_t  s_mcpwm_cmpr[LED_CTRL_MCPWM_CH_NUM];
static mcpwm_gen_handle_t   s_mcpwm_gen[LED_CTRL_MCPWM_CH_NUM];

/**
 * MCPWM 各通道 GPIO
 *
 * DRAM_ATTR 非必需 (无 ISR 访问)，但保持与通道表一致便于排查。
 */
static const int s_mcpwm_gpio[LED_CTRL_MCPWM_CH_NUM] = { 18, 19, 20, 21, 22, 23 };

/* ============================================================================
 * 工具函数
 * ========================================================================== */

/** 把 0-255 亮度线性映射到 0-LED_CTRL_DUTY_MAX (LEDC 12 位) */
static inline uint32_t scale_duty(uint32_t value)
{
    /* value 已保证 <= 255 */
    return (value * LED_CTRL_DUTY_MAX + 127) / 255;
}

/**
 * @brief 把 0-255 亮度映射为 MCPWM 比较值 (tick)
 *
 * MCPWM 计数器周期 = LED_CTRL_MCPWM_RES_HZ / freq_hz，
 * 比较值 = period * value / 255。
 */
static inline uint32_t scale_duty_mcpwm(uint32_t value, uint32_t period_ticks)
{
    return (value * period_ticks + 127) / 255;
}

/**
 * @brief 写入单路 PWM 占空比
 *
 * 前 6 路走 LEDC，后 6 路走 MCPWM，两者都是硬件外设。
 *
 * LEDC 写占空比用 ledc_set_duty() + ledc_update_duty() 两步:
 *   不用 ledc_set_duty_and_update()，因为该 API 内部依赖 fade 服务
 *   (未安装时返回 ESP_FAIL: "Fade service not installed")，
 *   而本项目不需要硬件渐变功能，装 fade 服务只会白占内存。
 *
 * 这两个 API 不是线程安全的，因此用 s_ledc_lock 串行化。
 *
 * @param ch     全局通道序号 (0-11)
 * @param duty   0-255 亮度
 * @param invert 是否反转
 */
static void write_channel(uint8_t ch, uint8_t duty, bool invert)
{
    if (ch < LED_CTRL_LEDC_CH_NUM) {
        /* --- LEDC 硬件通道 --- */
        uint32_t d = scale_duty(duty);
        if (invert) {
            d = LED_CTRL_DUTY_MAX - d;
        }

        if (xSemaphoreTake(s_ledc_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
            return;     /* 拿不到锁就跳过本次刷新，下个周期会重试 */
        }

        esp_err_t err = ledc_set_duty(LED_SPEED_MODE, (ledc_channel_t)ch, d);
        if (err == ESP_OK) {
            err = ledc_update_duty(LED_SPEED_MODE, (ledc_channel_t)ch);
        }
        xSemaphoreGive(s_ledc_lock);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "LEDC 通道 %u 写入失败: %s", ch, esp_err_to_name(err));
        }
    } else {
        /* --- MCPWM 硬件通道 --- */
        uint8_t  idx = ch - LED_CTRL_LEDC_CH_NUM;
        uint32_t period = LED_CTRL_MCPWM_RES_HZ / s_state.freq_hz;
        uint32_t d = scale_duty_mcpwm(duty, period);

        if (invert) {
            d = period - d;
        }

        esp_err_t err = mcpwm_comparator_set_compare_value(s_mcpwm_cmpr[idx], d);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "MCPWM 通道 %u 写入失败: %s", idx, esp_err_to_name(err));
        }
    }
}

/**
 * @brief 把灯珠颜色 (0-255) 写入对应通道
 *
 * @param idx       灯珠索引
 * @param r/g/b     已应用全局亮度的颜色值 (0-255)
 * @param invert    是否反转输出
 */
static void apply_led(uint8_t idx, uint8_t r, uint8_t g, uint8_t b, bool invert)
{
    const uint8_t duty[LED_CTRL_CH_PER_LED] = { r, g, b };

    for (int c = 0; c < LED_CTRL_CH_PER_LED; c++) {
        write_channel((uint8_t)LED_CH_OF(idx, c), duty[c], invert);
    }
}

/** 应用全局亮度缩放，返回 0-255 的实际亮度 */
static inline uint8_t apply_brightness(uint8_t value, uint8_t brightness)
{
    return (uint8_t)(((uint32_t)value * brightness + 127) / 255);
}

/* ============================================================================
 * 效果引擎
 * ========================================================================== */

/**
 * @brief 把 HSL 色相转换为 RGB (仅用于彩虹效果)
 *
 * @param hue_deg 色相 (0-360)
 * @param[out] r/g/b 输出 0-255
 */
static void hue_to_rgb(uint32_t hue_deg, uint8_t *r, uint8_t *g, uint8_t *b)
{
    /* 简化实现: 把色环六等分，每段做线性插值，饱和度和明度固定为满 */
    uint32_t h = hue_deg % 360;
    uint32_t seg = h / 60;              /* 0-5 */
    uint32_t frac = (h % 60) * 255 / 60; /* 0-255 */

    uint8_t up   = (uint8_t)frac;
    uint8_t down = (uint8_t)(255 - frac);

    switch (seg) {
    case 0:  *r = 255; *g = up;   *b = 0;   break;  /* 红 -> 黄 */
    case 1:  *r = down; *g = 255; *b = 0;   break;  /* 黄 -> 绿 */
    case 2:  *r = 0;   *g = 255; *b = up;   break;  /* 绿 -> 青 */
    case 3:  *r = 0;   *g = down; *b = 255; break;  /* 青 -> 蓝 */
    case 4:  *r = up;  *g = 0;   *b = 255;  break;  /* 蓝 -> 品红 */
    default: *r = 255; *g = 0;   *b = down; break;  /* 品红 -> 红 */
    }
}

/**
 * @brief 计算某颗灯珠在指定效果与相位下应显示的颜色
 *
 * 把「效果类型」作为参数传入，是为了同时支持两条路径:
 *   1. 单效果: 效果取自 s_state.effect[idx]
 *   2. 效果序列: 效果取自当前步骤的 steps[idx][step_idx].effect
 *
 * @param idx     灯珠索引 (仅用于彩虹/流水的错位计算)
 * @param effect  效果类型
 * @param phase   相位 (0.0 - 1.0)
 * @param[out] r/g/b 输出颜色 (0-255，未应用亮度)
 */
static void effect_color_ex(uint8_t idx, uint8_t effect, float phase,
                            uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = s_state.rgb[idx][LED_COLOR_R];
    *g = s_state.rgb[idx][LED_COLOR_G];
    *b = s_state.rgb[idx][LED_COLOR_B];

    switch ((led_effect_t)effect) {
    case LED_EFFECT_NONE:
    default:
        break;

    case LED_EFFECT_BREATH: {
        /*
         * 正弦呼吸: 亮度系数在 0.05 - 1.0 之间平滑起伏。
         * 保持 0.05 底值可避免完全熄灭时的视觉突变。
         */
        float k = 0.05f + 0.95f * (0.5f - 0.5f * cosf(phase * 2.0f * (float)M_PI));
        *r = (uint8_t)(*r * k);
        *g = (uint8_t)(*g * k);
        *b = (uint8_t)(*b * k);
        break;
    }

    case LED_EFFECT_BLINK: {
        /* 相位前半段点亮，后半段熄灭 */
        if (phase >= 0.5f) {
            *r = 0; *g = 0; *b = 0;
        }
        break;
    }

    case LED_EFFECT_RAINBOW: {
        /*
         * 彩虹: 色相沿灯珠顺序错开，形成流动感。
         * 每颗灯珠间隔 360/4 = 90 度。
         */
        uint32_t hue = (uint32_t)(phase * 360.0f) + (uint32_t)idx * 90u;
        uint8_t hr, hg, hb;
        hue_to_rgb(hue, &hr, &hg, &hb);

        /* 按灯珠设定颜色的最大分量做缩放，保留"整体色调"意图 */
        uint8_t peak = s_state.rgb[idx][LED_COLOR_R];
        if (s_state.rgb[idx][LED_COLOR_G] > peak) peak = s_state.rgb[idx][LED_COLOR_G];
        if (s_state.rgb[idx][LED_COLOR_B] > peak) peak = s_state.rgb[idx][LED_COLOR_B];
        if (peak == 0) {
            peak = 255;     /* 未设定颜色时以全亮彩虹显示 */
        }

        *r = (uint8_t)((uint32_t)hr * peak / 255);
        *g = (uint8_t)((uint32_t)hg * peak / 255);
        *b = (uint8_t)((uint32_t)hb * peak / 255);
        break;
    }

    case LED_EFFECT_CHASE: {
        /*
         * 流水: 4 颗灯珠均匀分布在一个周期内，
         * 每颗灯珠在自身窗口的前 1/4 时间点亮，其余时间渐暗。
         */
        float slot = 1.0f / (float)LED_CTRL_COUNT;
        float local = phase - (float)idx * slot;
        if (local < 0.0f) {
            local += 1.0f;
        }

        float k;
        if (local < slot * 0.5f) {
            k = 1.0f;
        } else if (local < slot) {
            /* 后半个窗口线性淡出 */
            k = 1.0f - (local - slot * 0.5f) / (slot * 0.5f);
        } else {
            k = 0.0f;
        }

        *r = (uint8_t)(*r * k);
        *g = (uint8_t)(*g * k);
        *b = (uint8_t)(*b * k);
        break;
    }
    }
}

/**
 * @brief 效果刷新任务
 *
 * 每 LED_REFRESH_MS 毫秒计算一次所有灯珠的目标颜色并写入 PWM。
 *
 * 每颗灯珠维护自己的相位累加器，按各自的周期取模，
 * 因此 D1 可以 2 秒呼吸、D2 可以 0.5 秒闪烁，互不影响。
 *
 * 若某颗灯珠设置了效果序列，则按步骤时长依次播放并循环，
 * 例如「1s 静态 -> 1s 闪烁 -> 1s 呼吸」。
 */
static void led_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(LED_REFRESH_MS);
    TickType_t last_wake = xTaskGetTickCount();

    while (s_task_run) {
        uint8_t  r[LED_CTRL_COUNT], g[LED_CTRL_COUNT], b[LED_CTRL_COUNT];
        uint8_t  brightness[LED_CTRL_COUNT];
        bool     enabled[LED_CTRL_COUNT];
        bool     invert;
        bool     off;

        /* --- 快照当前状态 (持锁时间尽量短) --- */
        if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        invert = s_state.invert;
        off    = s_state.off;
        memcpy(brightness, s_state.brightness, sizeof(brightness));
        memcpy(enabled,    s_state.enabled,    sizeof(enabled));

        for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
            uint8_t  eff;
            uint32_t p;

            if (s_state.step_count[i] > 0) {
                /* --- 序列模式: 按当前步骤的效果与周期播放 --- */
                uint8_t  n = s_state.step_count[i];
                if (s_step_idx[i] >= n) {
                    s_step_idx[i] = 0;      /* 越界保护 */
                }
                const led_step_t *st = &s_state.steps[i][s_step_idx[i]];

                eff = st->effect;
                p   = st->period_ms ? st->period_ms : st->duration_ms;
                if (p == 0) {
                    p = LED_DEFAULT_PERIOD_MS;
                }

                /* 相位按当前步骤的周期循环 */
                s_phase_ms[i] = (s_phase_ms[i] + LED_REFRESH_MS) % p;

                /*
                 * 步骤计时: 累加到 duration 就切下一步 (最后一步回到第一步)。
                 * 用 >= 而非 ==，避免刷新周期与 duration 不整除时漏判。
                 */
                s_step_elapsed[i] += LED_REFRESH_MS;
                if (s_step_elapsed[i] >= st->duration_ms) {
                    s_step_elapsed[i] = 0;
                    s_phase_ms[i]     = 0;      /* 新步骤从相位起点开始 */
                    s_step_idx[i]     = (uint8_t)((s_step_idx[i] + 1) % n);
                }
            } else {
                /* --- 单效果模式 --- */
                eff = s_state.effect[i];
                p   = s_state.period_ms[i] ? s_state.period_ms[i]
                                           : LED_DEFAULT_PERIOD_MS;
                s_phase_ms[i] = (s_phase_ms[i] + LED_REFRESH_MS) % p;
            }

            float phase = (float)s_phase_ms[i] / (float)p;
            effect_color_ex(i, eff, phase, &r[i], &g[i], &b[i]);
        }

        xSemaphoreGive(s_lock);

        /* --- 写入硬件 (不持锁，避免阻塞 HTTP 请求) --- */
        for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
            if (off || !enabled[i]) {
                apply_led(i, 0, 0, 0, invert);
                continue;
            }
            apply_led(i,
                      apply_brightness(r[i], brightness[i]),
                      apply_brightness(g[i], brightness[i]),
                      apply_brightness(b[i], brightness[i]),
                      invert);
        }

        vTaskDelayUntil(&last_wake, period);
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

/* ============================================================================
 * 初始化
 * ========================================================================== */

esp_err_t led_ctrl_init(void)
{
    if (s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /* --- 默认状态 --- */
    memset(&s_state, 0, sizeof(s_state));
    memset(s_phase_ms, 0, sizeof(s_phase_ms));
    memset(s_step_idx, 0, sizeof(s_step_idx));
    memset(s_step_elapsed, 0, sizeof(s_step_elapsed));

    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        s_state.brightness[i] = 255;
        s_state.effect[i]     = LED_EFFECT_NONE;
        s_state.period_ms[i]  = LED_DEFAULT_PERIOD_MS;
        s_state.enabled[i]    = true;
        s_state.step_count[i] = 0;      /* 默认无序列，走单效果 */
    }

    s_state.freq_hz    = LED_CTRL_DEFAULT_FREQ;
    s_state.invert     = false;
    s_state.off        = true;

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        return ESP_ERR_NO_MEM;
    }

    s_ledc_lock = xSemaphoreCreateMutex();
    if (s_ledc_lock == NULL) {
        ESP_LOGE(TAG, "创建 LEDC 互斥锁失败");
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* --- 配置 LEDC 定时器 (前 6 路共用) --- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LED_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num       = LED_TIMER_NUM,
        .freq_hz         = s_state.freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };

    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置 LEDC 定时器失败: %s", esp_err_to_name(err));
        goto fail;
    }

    /* --- 配置 LEDC 硬件通道 (前 6 路) --- */
    for (uint8_t ch = 0; ch < LED_CTRL_LEDC_CH_NUM; ch++) {
        uint8_t led   = ch / LED_CTRL_CH_PER_LED;
        int     color = ch % LED_CTRL_CH_PER_LED;

        ledc_channel_config_t ch_cfg = {
            .gpio_num   = led_ctrl_gpio[led][color],
            .speed_mode = LED_SPEED_MODE,
            .channel    = (ledc_channel_t)ch,
            .timer_sel  = LED_TIMER_NUM,
            .duty       = 0,
            .hpoint     = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags      = { .output_invert = 0 },
            .deconfigure = false,
        };

        err = ledc_channel_config(&ch_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "配置 LEDC 通道 %u 失败 (GPIO%d): %s",
                     ch, led_ctrl_gpio[led][color], esp_err_to_name(err));
            goto fail;
        }
    }

    /* --- 配置 MCPWM 定时器 (后 6 路共用) --- */
    uint32_t period_ticks = LED_CTRL_MCPWM_RES_HZ / s_state.freq_hz;

    mcpwm_timer_config_t mcpwm_timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = LED_CTRL_MCPWM_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = period_ticks,
    };

    err = mcpwm_new_timer(&mcpwm_timer_cfg, &s_mcpwm_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建 MCPWM 定时器失败: %s", esp_err_to_name(err));
        goto fail;
    }

    /* 必须先 enable 定时器，操作器才能连接它 */
    err = mcpwm_timer_enable(s_mcpwm_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "使能 MCPWM 定时器失败: %s", esp_err_to_name(err));
        goto fail;
    }

    /* --- 逐操作器配置 2 路 (比较器 + 生成器) --- */
    for (int op = 0; op < LED_MCPWM_OPER_NUM; op++) {
        mcpwm_operator_config_t oper_cfg = {
            .group_id = 0,
        };

        err = mcpwm_new_operator(&oper_cfg, &s_mcpwm_oper[op]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "创建 MCPWM 操作器 %d 失败: %s", op, esp_err_to_name(err));
            goto fail;
        }

        err = mcpwm_operator_connect_timer(s_mcpwm_oper[op], s_mcpwm_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "连接 MCPWM 操作器 %d 失败: %s", op, esp_err_to_name(err));
            goto fail;
        }

        for (int sub = 0; sub < LED_MCPWM_PER_OPER; sub++) {
            int idx = op * LED_MCPWM_PER_OPER + sub;

            /* --- 比较器 (决定占空比) --- */
            mcpwm_comparator_config_t cmpr_cfg = {
                .flags.update_cmp_on_tez = true,   /* 计数归零时更新，避免毛刺 */
            };

            err = mcpwm_new_comparator(s_mcpwm_oper[op], &cmpr_cfg,
                                       &s_mcpwm_cmpr[idx]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "创建 MCPWM 比较器 %d 失败: %s",
                         idx, esp_err_to_name(err));
                goto fail;
            }

            /* --- 生成器 (决定输出引脚) --- */
            mcpwm_generator_config_t gen_cfg = {
                .gen_gpio_num = s_mcpwm_gpio[idx],
            };

            err = mcpwm_new_generator(s_mcpwm_oper[op], &gen_cfg,
                                      &s_mcpwm_gen[idx]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "创建 MCPWM 生成器 %d 失败 (GPIO%d): %s",
                         idx, s_mcpwm_gpio[idx], esp_err_to_name(err));
                goto fail;
            }

            /*
             * 波形规则 1: 定时器计数归零 (周期开始) -> 输出高
             * 波形规则 2: 计数到达比较值 (占空比点) -> 输出低
             * 两条规则配合即得到标准 PWM。
             */
            err = mcpwm_generator_set_action_on_timer_event(s_mcpwm_gen[idx],
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                 MCPWM_TIMER_EVENT_EMPTY,
                                                 MCPWM_GEN_ACTION_HIGH));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "设置 MCPWM 生成器 %d 定时器动作失败: %s",
                         idx, esp_err_to_name(err));
                goto fail;
            }

            err = mcpwm_generator_set_action_on_compare_event(s_mcpwm_gen[idx],
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                   s_mcpwm_cmpr[idx],
                                                   MCPWM_GEN_ACTION_LOW));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "设置 MCPWM 生成器 %d 比较动作失败: %s",
                         idx, esp_err_to_name(err));
                goto fail;
            }

            /* 初始占空比 0 (熄灭) */
            mcpwm_comparator_set_compare_value(s_mcpwm_cmpr[idx], 0);
        }
    }

    /* --- 启动 MCPWM 定时器 --- */
    /*
     * 用 MCPWM_TIMER_START_NO_STOP: 启动后持续计数不自动停止。
     * (START_STOP_EMPTY 会在下一个周期结束时停住，只适合单次输出。)
     */
    err = mcpwm_timer_start_stop(s_mcpwm_timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动 MCPWM 定时器失败: %s", esp_err_to_name(err));
        goto fail;
    }

    /* --- 启动效果任务 --- */
    s_task_run = true;
    BaseType_t ok = xTaskCreate(led_task, "led_effect", LED_TASK_STACK,
                                NULL, LED_TASK_PRIO, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建效果任务失败");
        s_task_run = false;
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    s_inited = true;

    ESP_LOGI(TAG, "灯珠控制已初始化: %d 颗 x 3 色 = %d 路 PWM",
             LED_CTRL_COUNT, LED_CTRL_CH_TOTAL);
    ESP_LOGI(TAG, "  前 %d 路: LEDC  硬件 PWM (%d 位, %" PRIu32 " Hz)",
             LED_CTRL_LEDC_CH_NUM, LED_CTRL_RES_BITS, s_state.freq_hz);
    ESP_LOGI(TAG, "  后 %d 路: MCPWM 硬件 PWM (%d 操作器 x %d, %" PRIu32 " Hz)",
             LED_CTRL_MCPWM_CH_NUM, LED_MCPWM_OPER_NUM, LED_MCPWM_PER_OPER,
             s_state.freq_hz);
    ESP_LOGI(TAG, "  运行期 CPU 占用: 0%% (全部硬件 PWM)");
    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        ESP_LOGI(TAG, "  %s (%u, %u) mm  R=IO%d G=IO%d B=IO%d",
                 led_ctrl_names[i], led_ctrl_pos_x[i], led_ctrl_pos_y[i],
                 led_ctrl_gpio[i][LED_COLOR_R],
                 led_ctrl_gpio[i][LED_COLOR_G],
                 led_ctrl_gpio[i][LED_COLOR_B]);
    }

    return ESP_OK;

fail:
    /* 清理 MCPWM 资源 (逆序释放) */
    for (int i = 0; i < LED_CTRL_MCPWM_CH_NUM; i++) {
        if (s_mcpwm_gen[i] != NULL) {
            mcpwm_del_generator(s_mcpwm_gen[i]);
            s_mcpwm_gen[i] = NULL;
        }
        if (s_mcpwm_cmpr[i] != NULL) {
            mcpwm_del_comparator(s_mcpwm_cmpr[i]);
            s_mcpwm_cmpr[i] = NULL;
        }
    }
    for (int i = 0; i < LED_MCPWM_OPER_NUM; i++) {
        if (s_mcpwm_oper[i] != NULL) {
            mcpwm_del_operator(s_mcpwm_oper[i]);
            s_mcpwm_oper[i] = NULL;
        }
    }
    if (s_mcpwm_timer != NULL) {
        mcpwm_timer_disable(s_mcpwm_timer);
        mcpwm_del_timer(s_mcpwm_timer);
        s_mcpwm_timer = NULL;
    }

    if (s_ledc_lock != NULL) {
        vSemaphoreDelete(s_ledc_lock);
        s_ledc_lock = NULL;
    }

    if (s_lock != NULL) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
    }
    return err;
}

esp_err_t led_ctrl_deinit(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 停止效果任务 */
    if (s_task != NULL) {
        s_task_run = false;
        /* 等待任务自行退出 (最多 200 ms) */
        for (int i = 0; i < 20 && s_task != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    /* 关闭所有 LEDC 通道输出 */
    for (uint8_t ch = 0; ch < LED_CTRL_LEDC_CH_NUM; ch++) {
        ledc_stop(LED_SPEED_MODE, (ledc_channel_t)ch, 0);
    }

    /* 关闭 MCPWM: 比较值置 0 后停定时器 */
    for (int i = 0; i < LED_CTRL_MCPWM_CH_NUM; i++) {
        if (s_mcpwm_cmpr[i] != NULL) {
            mcpwm_comparator_set_compare_value(s_mcpwm_cmpr[i], 0);
        }
    }
    if (s_mcpwm_timer != NULL) {
        mcpwm_timer_start_stop(s_mcpwm_timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(s_mcpwm_timer);
    }

    /* 释放 MCPWM 资源 */
    for (int i = 0; i < LED_CTRL_MCPWM_CH_NUM; i++) {
        if (s_mcpwm_gen[i] != NULL) {
            mcpwm_del_generator(s_mcpwm_gen[i]);
            s_mcpwm_gen[i] = NULL;
        }
        if (s_mcpwm_cmpr[i] != NULL) {
            mcpwm_del_comparator(s_mcpwm_cmpr[i]);
            s_mcpwm_cmpr[i] = NULL;
        }
    }
    for (int i = 0; i < LED_MCPWM_OPER_NUM; i++) {
        if (s_mcpwm_oper[i] != NULL) {
            mcpwm_del_operator(s_mcpwm_oper[i]);
            s_mcpwm_oper[i] = NULL;
        }
    }
    if (s_mcpwm_timer != NULL) {
        mcpwm_del_timer(s_mcpwm_timer);
        s_mcpwm_timer = NULL;
    }

    if (s_ledc_lock != NULL) {
        vSemaphoreDelete(s_ledc_lock);
        s_ledc_lock = NULL;
    }

    if (s_lock != NULL) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
    }
    s_inited = false;

    ESP_LOGI(TAG, "灯珠控制已反初始化");
    return ESP_OK;
}

/* ============================================================================
 * 颜色控制
 * ========================================================================== */

esp_err_t led_ctrl_set_rgb(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx >= LED_CTRL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_state.rgb[idx][LED_COLOR_R] = r;
    s_state.rgb[idx][LED_COLOR_G] = g;
    s_state.rgb[idx][LED_COLOR_B] = b;

    /* 只要设置了非零颜色，就认为用户希望点亮 */
    if (r || g || b) {
        s_state.off = false;
    }

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_get_rgb(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (idx >= LED_CTRL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (r) *r = s_state.rgb[idx][LED_COLOR_R];
    if (g) *g = s_state.rgb[idx][LED_COLOR_G];
    if (b) *b = s_state.rgb[idx][LED_COLOR_B];

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        s_state.rgb[i][LED_COLOR_R] = r;
        s_state.rgb[i][LED_COLOR_G] = g;
        s_state.rgb[i][LED_COLOR_B] = b;
    }
    if (r || g || b) {
        s_state.off = false;
    }

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_all_off(void)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_state.off = true;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_set_brightness(uint8_t brightness)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        s_state.brightness[i] = brightness;
    }

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

uint8_t led_ctrl_get_brightness(void)
{
    uint8_t v = 0;
    if (s_lock == NULL) {
        return 0;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        /* 汇总值取最大亮度，代表"整体亮度" */
        for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
            if (s_state.brightness[i] > v) {
                v = s_state.brightness[i];
            }
        }
        xSemaphoreGive(s_lock);
    }
    return v;
}

esp_err_t led_ctrl_set_led_brightness(uint8_t idx, uint8_t brightness)
{
    if (idx >= LED_CTRL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_state.brightness[idx] = brightness;

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

uint8_t led_ctrl_get_led_brightness(uint8_t idx)
{
    uint8_t v = 0;
    if (idx >= LED_CTRL_COUNT || s_lock == NULL) {
        return 0;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        v = s_state.brightness[idx];
        xSemaphoreGive(s_lock);
    }
    return v;
}

/* ============================================================================
 * 效果控制
 * ========================================================================== */

esp_err_t led_ctrl_set_effect(led_effect_t effect, uint32_t period_ms)
{
    if (effect >= LED_EFFECT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (period_ms != 0 && (period_ms < 100 || period_ms > 60000)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        s_state.effect[i] = (uint8_t)effect;
        if (period_ms != 0) {
            s_state.period_ms[i] = period_ms;
        }
        /* 切换效果时重置相位，使所有灯珠从周期起点同步开始 */
        s_phase_ms[i] = 0;
    }

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_get_effect(led_effect_t *effect, uint32_t *period_ms)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 汇总值取第一颗灯珠，代表"整体效果" */
    if (effect)    *effect    = (led_effect_t)s_state.effect[0];
    if (period_ms) *period_ms = s_state.period_ms[0];

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_set_led_effect(uint8_t idx, led_effect_t effect,
                                  uint32_t period_ms)
{
    if (idx >= LED_CTRL_COUNT || effect >= LED_EFFECT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (period_ms != 0 && (period_ms < 100 || period_ms > 60000)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_state.effect[idx] = (uint8_t)effect;
    if (period_ms != 0) {
        s_state.period_ms[idx] = period_ms;
    }

    xSemaphoreGive(s_lock);

    /*
     * 切换效果时重置相位，让新效果从周期起点开始，
     * 避免沿用旧相位造成首帧颜色跳变。
     */
    s_phase_ms[idx] = 0;

    return ESP_OK;
}

esp_err_t led_ctrl_get_led_effect(uint8_t idx, led_effect_t *effect,
                                  uint32_t *period_ms)
{
    if (idx >= LED_CTRL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (effect)    *effect    = (led_effect_t)s_state.effect[idx];
    if (period_ms) *period_ms = s_state.period_ms[idx];

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_set_led_enabled(uint8_t idx, bool enabled)
{
    if (idx >= LED_CTRL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_state.enabled[idx] = enabled;

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool led_ctrl_get_led_enabled(uint8_t idx)
{
    bool v = false;
    if (idx >= LED_CTRL_COUNT || s_lock == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        v = s_state.enabled[idx];
        xSemaphoreGive(s_lock);
    }
    return v;
}

esp_err_t led_ctrl_set_led_state(uint8_t idx,
                                 const uint8_t *r, const uint8_t *g,
                                 const uint8_t *b,
                                 const uint8_t *brightness,
                                 const led_effect_t *effect,
                                 const uint32_t *period_ms,
                                 const bool *enabled)
{
    if (idx >= LED_CTRL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (effect != NULL && *effect >= LED_EFFECT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (period_ms != NULL && *period_ms != 0 &&
        (*period_ms < 100 || *period_ms > 60000)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 颜色三通道一起更新，避免出现混合色 */
    if (r != NULL) s_state.rgb[idx][LED_COLOR_R] = *r;
    if (g != NULL) s_state.rgb[idx][LED_COLOR_G] = *g;
    if (b != NULL) s_state.rgb[idx][LED_COLOR_B] = *b;

    if (brightness != NULL) s_state.brightness[idx] = *brightness;
    if (effect != NULL)     s_state.effect[idx]     = (uint8_t)*effect;
    if (period_ms != NULL && *period_ms != 0) {
        s_state.period_ms[idx] = *period_ms;
    }
    if (enabled != NULL)    s_state.enabled[idx]    = *enabled;

    /* 设置了非零颜色或显式启用，就认为用户希望点亮 */
    if ((r != NULL && (*r || (g && *g) || (b && *b))) ||
        (enabled != NULL && *enabled)) {
        s_state.off = false;
    }

    xSemaphoreGive(s_lock);

    /* 效果变更时重置相位，避免首帧颜色跳变 */
    if (effect != NULL) {
        s_phase_ms[idx] = 0;
    }

    return ESP_OK;
}

led_effect_t led_ctrl_effect_from_name(const char *name)
{
    if (name == NULL) {
        return LED_EFFECT_MAX;
    }
    for (int i = 0; i < LED_EFFECT_MAX; i++) {
        if (strcasecmp(name, led_effect_names[i]) == 0) {
            return (led_effect_t)i;
        }
    }
    return LED_EFFECT_MAX;
}

/* ============================================================================
 * 效果序列
 * ========================================================================== */

/** 校验单个步骤是否合法 */
static bool step_valid(const led_step_t *st)
{
    if (st->effect >= LED_EFFECT_MAX) {
        return false;
    }
    /* 步骤时长: 10 ms - 10 分钟 */
    if (st->duration_ms < 10 || st->duration_ms > 600000) {
        return false;
    }
    /* 步骤内周期: 0 (表示与时长相同) 或 100 ms - 60 s */
    if (st->period_ms != 0 &&
        (st->period_ms < 100 || st->period_ms > 60000)) {
        return false;
    }
    return true;
}

esp_err_t led_ctrl_set_led_sequence(uint8_t idx, const led_step_t *steps,
                                    uint8_t step_count)
{
    if (idx >= LED_CTRL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (step_count > LED_SEQ_MAX_STEPS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (step_count > 0 && steps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < step_count; i++) {
        if (!step_valid(&steps[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (step_count > 0) {
        memcpy(s_state.steps[idx], steps, sizeof(led_step_t) * step_count);
    }
    s_state.step_count[idx] = step_count;

    xSemaphoreGive(s_lock);

    /* 从第一步重新开始播放 */
    s_step_idx[idx]     = 0;
    s_step_elapsed[idx] = 0;
    s_phase_ms[idx]     = 0;

    return ESP_OK;
}

esp_err_t led_ctrl_get_led_sequence(uint8_t idx, led_step_t *steps,
                                    uint8_t *step_count)
{
    if (idx >= LED_CTRL_COUNT || step_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t n = s_state.step_count[idx];

    if (steps != NULL) {
        if (*step_count < n) {
            xSemaphoreGive(s_lock);
            return ESP_ERR_INVALID_SIZE;
        }
        if (n > 0) {
            memcpy(steps, s_state.steps[idx], sizeof(led_step_t) * n);
        }
    }
    *step_count = n;

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t led_ctrl_clear_led_sequence(uint8_t idx)
{
    return led_ctrl_set_led_sequence(idx, NULL, 0);
}

esp_err_t led_ctrl_set_sequence(const led_step_t *steps, uint8_t step_count)
{
    if (step_count > LED_SEQ_MAX_STEPS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (step_count > 0 && steps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < step_count; i++) {
        if (!step_valid(&steps[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    esp_err_t err = ESP_OK;
    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        esp_err_t e = led_ctrl_set_led_sequence(i, steps, step_count);
        if (e != ESP_OK) {
            err = e;    /* 记录首个错误，但继续设置其余灯珠 */
        }
    }
    return err;
}

esp_err_t led_ctrl_clear_sequence(void)
{
    return led_ctrl_set_sequence(NULL, 0);
}

/* ============================================================================
 * PWM 参数
 * ========================================================================== */

esp_err_t led_ctrl_set_freq(uint32_t freq_hz)
{
    if (freq_hz < 100 || freq_hz > 40000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * LEDC 与 MCPWM 两个外设同步改频。
     *
     * LEDC: 直接改定时器频率。先暂停定时器，改完恢复，
     *       避免中途出现异常占空比。
     *
     * MCPWM: 计数器分辨率固定 (1 MHz)，改频即改周期 tick 数。
     *        周期变化后原比较值会失去意义，因此改完立即按当前
     *        颜色重算占空比 (下次效果任务刷新时自然生效)。
     */
    ledc_timer_pause(LED_SPEED_MODE, LED_TIMER_NUM);
    esp_err_t err = ledc_set_freq(LED_SPEED_MODE, LED_TIMER_NUM, freq_hz);
    ledc_timer_resume(LED_SPEED_MODE, LED_TIMER_NUM);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "设置 LEDC 频率失败: %s", esp_err_to_name(err));
        return err;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_state.freq_hz = freq_hz;
        xSemaphoreGive(s_lock);
    }

    /* MCPWM 周期同步更新 */
    if (s_mcpwm_timer != NULL) {
        uint32_t period_ticks = LED_CTRL_MCPWM_RES_HZ / freq_hz;
        err = mcpwm_timer_set_period(s_mcpwm_timer, period_ticks);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "设置 MCPWM 周期失败: %s", esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "PWM 频率已设为 %" PRIu32 " Hz (LEDC + MCPWM)", freq_hz);
    return ESP_OK;
}

uint32_t led_ctrl_get_freq(void)
{
    uint32_t v = 0;
    if (s_lock == NULL) {
        return 0;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        v = s_state.freq_hz;
        xSemaphoreGive(s_lock);
    }
    return v;
}

esp_err_t led_ctrl_set_invert(bool invert)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_state.invert = invert;
    xSemaphoreGive(s_lock);

    /*
     * 反转在 write_channel() 中按 (满量程 - 占空比) 实现，
     * 下次效果任务刷新时即生效，无需额外操作硬件。
     */
    return ESP_OK;
}

bool led_ctrl_get_invert(void)
{
    bool v = false;
    if (s_lock == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        v = s_state.invert;
        xSemaphoreGive(s_lock);
    }
    return v;
}

/* ============================================================================
 * JSON 输出
 * ========================================================================== */

esp_err_t led_ctrl_to_json(char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len < 256) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t  rgb[LED_CTRL_COUNT][LED_CTRL_CH_PER_LED];
    uint8_t  brightness[LED_CTRL_COUNT];
    uint8_t  effect[LED_CTRL_COUNT];
    uint32_t period_ms[LED_CTRL_COUNT];
    bool     enabled[LED_CTRL_COUNT];
    uint8_t  step_count[LED_CTRL_COUNT];
    led_step_t steps[LED_CTRL_COUNT][LED_SEQ_MAX_STEPS];
    uint32_t freq_hz;
    bool     invert;
    bool     off;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(rgb, s_state.rgb, sizeof(rgb));
    memcpy(brightness, s_state.brightness, sizeof(brightness));
    memcpy(effect, s_state.effect, sizeof(effect));
    memcpy(period_ms, s_state.period_ms, sizeof(period_ms));
    memcpy(enabled, s_state.enabled, sizeof(enabled));
    memcpy(step_count, s_state.step_count, sizeof(step_count));
    memcpy(steps, s_state.steps, sizeof(steps));
    freq_hz = s_state.freq_hz;
    invert  = s_state.invert;
    off     = s_state.off;

    xSemaphoreGive(s_lock);

    int n = snprintf(buf, buf_len,
                     "{\"count\":%d,\"leds\":[", LED_CTRL_COUNT);

    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        n += snprintf(buf + n, buf_len - (size_t)n,
                      "%s{\"id\":%u,\"name\":\"%s\",\"pos\":\"(%u, %u) mm\","
                      "\"gpio\":{\"r\":%d,\"g\":%d,\"b\":%d},"
                      "\"r\":%u,\"g\":%u,\"b\":%u,"
                      "\"brightness\":%u,\"effect\":\"%s\","
                      "\"period\":%" PRIu32 ",\"enabled\":%s,"
                      "\"steps\":[",
                      i ? "," : "",
                      (unsigned)(i + 1), led_ctrl_names[i],
                      led_ctrl_pos_x[i], led_ctrl_pos_y[i],
                      led_ctrl_gpio[i][LED_COLOR_R],
                      led_ctrl_gpio[i][LED_COLOR_G],
                      led_ctrl_gpio[i][LED_COLOR_B],
                      rgb[i][LED_COLOR_R], rgb[i][LED_COLOR_G], rgb[i][LED_COLOR_B],
                      brightness[i],
                      (effect[i] < LED_EFFECT_MAX)
                          ? led_effect_names[effect[i]] : "none",
                      period_ms[i],
                      enabled[i] ? "true" : "false");

        if (n >= (int)buf_len) {
            return ESP_ERR_INVALID_SIZE;
        }

        /* 效果序列: 每步 {effect, duration, period} */
        for (uint8_t k = 0; k < step_count[i]; k++) {
            n += snprintf(buf + n, buf_len - (size_t)n,
                          "%s{\"effect\":\"%s\",\"duration\":%" PRIu32 ","
                          "\"period\":%" PRIu32 "}",
                          k ? "," : "",
                          (steps[i][k].effect < LED_EFFECT_MAX)
                              ? led_effect_names[steps[i][k].effect] : "none",
                          steps[i][k].duration_ms,
                          steps[i][k].period_ms);

            if (n >= (int)buf_len) {
                return ESP_ERR_INVALID_SIZE;
            }
        }

        n += snprintf(buf + n, buf_len - (size_t)n, "]}");

        if (n >= (int)buf_len) {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    /*
     * 顶层 brightness/effect/period 为汇总值 (取首颗灯珠)，
     * 供旧前端与"一键同步"控件回显使用；逐颗真实值在 leds[] 内。
     */
    n += snprintf(buf + n, buf_len - (size_t)n,
                  "],\"brightness\":%u,\"effect\":\"%s\",\"period\":%" PRIu32 ","
                  "\"freq\":%" PRIu32 ",\"invert\":%s,\"off\":%s}",
                  brightness[0],
                  (effect[0] < LED_EFFECT_MAX) ? led_effect_names[effect[0]] : "none",
                  period_ms[0], freq_hz,
                  invert ? "true" : "false",
                  off ? "true" : "false");

    if (n >= (int)buf_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

/* ============================================================================
 * 配置持久化: 保存 / 加载
 * ========================================================================== */

/**
 * @brief 把当前灯珠状态保存到 NVS
 *
 * 保存内容: 每颗灯珠的颜色、亮度、效果、周期、使能、效果序列，
 *           以及全局 PWM 频率、输出反转、熄灭标志。
 *
 * 上电时由 led_ctrl_load() 自动恢复，因此调用本函数后即使断电，
 * 下次启动也会回到当前状态。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE 模块未初始化
 *         其他 NVS 错误
 */
esp_err_t led_ctrl_save(void)
{
    if (!s_inited || s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    led_nvs_blob_t *blob = malloc(sizeof(led_nvs_blob_t));
    if (blob == NULL) {
        ESP_LOGE(TAG, "保存配置: 内存不足");
        return ESP_ERR_NO_MEM;
    }

    /* --- 在锁保护下拷贝当前状态 --- */
    memset(blob, 0, sizeof(*blob));
    blob->version = LED_NVS_VERSION;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        free(blob);
        return ESP_ERR_TIMEOUT;
    }

    memcpy(blob->rgb,        s_state.rgb,        sizeof(blob->rgb));
    memcpy(blob->brightness, s_state.brightness, sizeof(blob->brightness));
    memcpy(blob->effect,     s_state.effect,     sizeof(blob->effect));
    memcpy(blob->period_ms,  s_state.period_ms,  sizeof(blob->period_ms));
    memcpy(blob->steps,      s_state.steps,      sizeof(blob->steps));
    memcpy(blob->step_count, s_state.step_count, sizeof(blob->step_count));
    blob->freq_hz = s_state.freq_hz;
    blob->invert  = s_state.invert ? 1 : 0;
    blob->off     = s_state.off ? 1 : 0;

    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        blob->enabled[i] = s_state.enabled[i] ? 1 : 0;
    }

    xSemaphoreGive(s_lock);

    /* --- 写入 NVS --- */
    nvs_handle_t h;
    esp_err_t err = nvs_open(LED_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "打开 NVS 命名空间失败: %s", esp_err_to_name(err));
        free(blob);
        return err;
    }

    err = nvs_set_blob(h, LED_NVS_KEY_STATE, blob, sizeof(*blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    free(blob);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写入灯珠配置失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "灯珠配置已保存到 NVS");
    }
    return err;
}

/**
 * @brief 从 NVS 加载灯珠状态并立即生效
 *
 * 在 led_ctrl_init() 之后调用。若 NVS 中无有效记录 (首次启动或
 * 版本不符)，保持默认状态并返回 ESP_ERR_NOT_FOUND。
 *
 * 加载后所有灯珠按保存的参数输出 (含效果与序列)。
 *
 * @return ESP_OK 成功恢复
 *         ESP_ERR_NOT_FOUND 无有效保存记录 (使用默认值)
 *         ESP_ERR_INVALID_STATE 模块未初始化
 *         其他 NVS 错误
 */
esp_err_t led_ctrl_load(void)
{
    if (!s_inited || s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(LED_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS 中无灯珠配置，使用默认值");
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "打开 NVS 命名空间失败: %s", esp_err_to_name(err));
        return err;
    }

    /* --- 先查大小，避免读到不匹配的旧数据 --- */
    size_t len = 0;
    err = nvs_get_blob(h, LED_NVS_KEY_STATE, NULL, &len);
    if (err != ESP_OK || len != sizeof(led_nvs_blob_t)) {
        nvs_close(h);
        ESP_LOGW(TAG, "灯珠配置缺失或长度不符 (需 %u 字节)，使用默认值",
                 (unsigned)sizeof(led_nvs_blob_t));
        return ESP_ERR_NOT_FOUND;
    }

    led_nvs_blob_t *blob = malloc(sizeof(led_nvs_blob_t));
    if (blob == NULL) {
        nvs_close(h);
        return ESP_ERR_NO_MEM;
    }

    err = nvs_get_blob(h, LED_NVS_KEY_STATE, blob, &len);
    nvs_close(h);

    if (err != ESP_OK) {
        free(blob);
        ESP_LOGW(TAG, "读取灯珠配置失败: %s", esp_err_to_name(err));
        return err;
    }

    if (blob->version != LED_NVS_VERSION) {
        free(blob);
        ESP_LOGW(TAG, "灯珠配置版本不符 (存 %u 需 %u)，使用默认值",
                 (unsigned)blob->version, (unsigned)LED_NVS_VERSION);
        return ESP_ERR_NOT_FOUND;
    }

    /* --- 应用: 先改硬件频率/反转，再逐颗恢复 --- */
    if (blob->freq_hz >= 100 && blob->freq_hz <= 40000) {
        led_ctrl_set_freq(blob->freq_hz);
    }
    led_ctrl_set_invert(blob->invert != 0);

    for (uint8_t i = 0; i < LED_CTRL_COUNT; i++) {
        uint8_t eff = blob->effect[i];
        if (eff >= LED_EFFECT_MAX) {
            eff = LED_EFFECT_NONE;
        }
        led_effect_t eff_v = (led_effect_t)eff;
        bool en_v = (blob->enabled[i] != 0);

        led_ctrl_set_led_state(i,
                               &blob->rgb[i][LED_COLOR_R],
                               &blob->rgb[i][LED_COLOR_G],
                               &blob->rgb[i][LED_COLOR_B],
                               &blob->brightness[i],
                               &eff_v,
                               &blob->period_ms[i],
                               &en_v);

        /* 效果序列 (step_count 为 0 时清除) */
        uint8_t sc = blob->step_count[i];
        if (sc > LED_SEQ_MAX_STEPS) {
            sc = LED_SEQ_MAX_STEPS;
        }
        led_ctrl_set_led_sequence(i, sc ? blob->steps[i] : NULL, sc);
    }

    /*
     * 恢复熄灭标志。
     *
     * s_state.off 为 true 时所有灯珠输出全灭 (但保留颜色/效果设置)，
     * 因此需要与实际颜色分开处理: 这里直接操作内部标志。
     */
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) == pdTRUE) {
        s_state.off = (blob->off != 0);
        xSemaphoreGive(s_lock);
    }

    free(blob);

    ESP_LOGI(TAG, "已从 NVS 恢复灯珠配置 (频率=%" PRIu32 " Hz)",
             led_ctrl_get_freq());
    return ESP_OK;
}

/**
 * @brief 清除已保存的灯珠配置
 *
 * 删除 NVS 中的记录，但不改变当前运行状态。下次上电将使用默认值。
 *
 * @return ESP_OK 成功 (含本就无记录)
 */
esp_err_t led_ctrl_clear_saved(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(LED_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(h, LED_NVS_KEY_STATE);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "已清除保存的灯珠配置");
    }
    return err;
}
