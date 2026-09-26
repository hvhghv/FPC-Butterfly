/*
 * 蝴蝶板灯控应用 — IP5108 电量计驱动实现
 *
 * 协议细节见 battery_ip5108.h。
 *
 * 【引脚复用: IO12/IO13 从 USB 拿回来】
 *
 * ESP32-C6 的 IO12/IO13 复用为 USB Serial/JTAG 的 D-/D+。IAP 程序会把
 * 它们配置为 USB 串口（终端 + 烧录），用户程序启动时这两个引脚仍处于
 * USB 外设控制之下，直接当普通 GPIO/I2C 用会失败。
 *
 * 释放步骤（缺一不可）:
 *   1. usb_serial_jtag_driver_uninstall()  卸载 USB 串口驱动
 *   2. usb_serial_jtag_stop()               停止 USB Serial/JTAG 外设
 *   3. 重新配置 GPIO 为 I2C 功能
 *
 * ⚠️ 副作用: 释放后 USB 串口终端失效。本应用通过 WiFi/BLE 提供控制通道，
 *    日志仍可从 UART0 输出，因此不受影响。
 *
 * 【I2C 访问流程】
 *
 *   IP5108 在 sleep->wake 时会采样 SCL/SDA 是否上拉到 VREG 来决定工作
 *   模式（I2C / LED 指示）。若未接 INT 引脚，只能"尽力而为"地直接探测:
 *   探测成功即说明已在 I2C 模式。
 */

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "battery_ip5108.h"

static const char *TAG = "battery";

/* ============================================================================
 * IP5108 寄存器地址
 * ========================================================================== */

/* --- 只读状态寄存器 --- */
#define REG_READ0B          0x71    /*!< 充电状态 + 超时标志 */
#define REG_READ1           0x72    /*!< 负载 / 输入状态 */
#define REG_READ2           0x77    /*!< 按键状态 */

/* --- ADC 数据寄存器 (各 2 字节，低字节在前) --- */
#define REG_BATVADC_DAT0    0xA2    /*!< 电池电压 低字节 */
#define REG_BATVADC_DAT1    0xA3    /*!< 电池电压 高字节 */
#define REG_BATIADC_DAT0    0xA4    /*!< 电池电流 低字节 */
#define REG_BATIADC_DAT1    0xA5    /*!< 电池电流 高字节 */
#define REG_BATOCV_DAT0     0xA8    /*!< 开路电压 低字节 */
#define REG_BATOCV_DAT1     0xA9    /*!< 开路电压 高字节 */

/* ============================================================================
 * ADC 换算系数 (来自数据手册)
 * ========================================================================== */

/** 电压 ADC 每 LSB 对应的毫伏数 */
#define ADC_V_LSB_MV        0.26855f

/** 电压 ADC 零点 (mV) */
#define ADC_V_ZERO_MV       2600.0f

/** 电流 ADC 每 LSB 对应的微安数 (0.745985 mA) */
#define ADC_I_LSB_MA        0.745985f

/** 电压 ADC 的"无电池"哨兵值 (mV) */
#define ADC_V_NO_BATTERY    4868

/* ============================================================================
 * 内部状态
 * ========================================================================== */

/** I2C 主机总线句柄 */
static i2c_master_bus_handle_t s_bus  = NULL;

/** IP5108 设备句柄 */
static i2c_master_dev_handle_t s_dev  = NULL;

/** 是否已成功初始化并探测到芯片 */
static bool s_ready = false;

/* ============================================================================
 * I2C 底层
 * ========================================================================== */

/**
 * @brief 读一个寄存器
 *
 * @param reg 寄存器地址
 * @param[out] val 输出值
 * @return ESP_OK 成功
 */
static esp_err_t reg_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, 100);
}

/**
 * @brief 连续读多个寄存器
 *
 * @param reg    起始寄存器地址
 * @param[out] buf 输出缓冲区
 * @param len    读取字节数
 * @return ESP_OK 成功
 */
static esp_err_t reg_read_multi(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, 100);
}

/**
 * @brief 读一对 ADC 寄存器并合成 16 位原始值
 *
 * 低字节在前 (DAT0 = 低 8 位, DAT1 = 高 6 位)。
 *
 * @param reg0 低字节寄存器地址
 * @param[out] raw 输出原始值 (低 8 位 + 高 6 位 << 8)
 * @return ESP_OK 成功
 */
static esp_err_t adc_read_pair(uint8_t reg0, uint16_t *raw)
{
    uint8_t b[2];
    esp_err_t err = reg_read_multi(reg0, b, sizeof(b));
    if (err != ESP_OK) {
        return err;
    }
    *raw = (uint16_t)b[0] | ((uint16_t)(b[1] & 0x3F) << 8);
    return ESP_OK;
}

/* ============================================================================
 * ADC 解码 (数据手册公式)
 * ========================================================================== */

/**
 * @brief 解码电压 ADC 原始值 -> 伏特
 *
 * 数据手册公式 (high 只取低 6 位):
 *   若 bit5 置位 (负值区):
 *       mV = 2600 - ((~low) + (~(high & 0x1F)) * 256 + 1) * 0.26855
 *   否则:
 *       mV = 2600 + (low + high * 256) * 0.26855
 *
 * 特殊值 4868mV 表示未接电池，此时返回 0。
 *
 * @param raw 16 位原始值 (低 8 位 + 高 6 位 << 8)
 * @return 电压 (V)
 */
static float decode_voltage(uint16_t raw)
{
    uint8_t low  = (uint8_t)(raw & 0xFF);
    uint8_t high = (uint8_t)((raw >> 8) & 0x3F);

    float mv;

    if (high & 0x20) {
        /* 负值区: 按补码还原 */
        uint32_t inv = (uint32_t)(uint8_t)(~low) +
                       (uint32_t)((~high) & 0x1F) * 256u + 1u;
        mv = ADC_V_ZERO_MV - (float)inv * ADC_V_LSB_MV;
    } else {
        mv = ADC_V_ZERO_MV + (float)((uint32_t)low +
                                     (uint32_t)high * 256u) * ADC_V_LSB_MV;
    }

    /* 无电池哨兵值 */
    if ((int)(mv + 0.5f) == ADC_V_NO_BATTERY) {
        return 0.0f;
    }

    return mv / 1000.0f;
}

/**
 * @brief 解码电流 ADC 原始值 -> 安培
 *
 * 数据手册公式 (high 只取低 6 位):
 *   若 bit5 置位 (放电，负值):
 *       mA = -((( (~(high & 0x1F) & 0x1F) * 256 + (~low) + 1 )) * 0.745985)
 *   否则 (充电，正值):
 *       mA = (high * 256 + low) * 0.745985
 *
 * @param raw 16 位原始值
 * @return 电流 (A)，充电为正 / 放电为负
 */
static float decode_current(uint16_t raw)
{
    uint8_t low  = (uint8_t)(raw & 0xFF);
    uint8_t high = (uint8_t)((raw >> 8) & 0x3F);

    float ma;

    if (high & 0x20) {
        /* 放电 (负值): 按补码还原 */
        uint32_t inv = (uint32_t)((~high) & 0x1F) * 256u +
                       (uint32_t)(uint8_t)(~low) + 1u;
        ma = -(float)inv * ADC_I_LSB_MA;
    } else {
        ma = (float)((uint32_t)low + (uint32_t)high * 256u) * ADC_I_LSB_MA;
    }

    return ma / 1000.0f;
}

/* ============================================================================
 * 电量百分比估算
 *
 * 用开路电压 (OCV) 查分段线性曲线。
 *
 * 为什么用 OCV 而非 BATVADC: 数据手册中 BATOCV = BATVADC + BATIADC * 内阻，
 * 即芯片已在 OCV 里补偿了充放电电流引起的 IR 压降。因此同一条 OCV->SOC
 * 曲线在空闲/涓流/CC/CV 各阶段都适用，不会在充电阶段切换时跳变。
 *
 * 曲线为 1S 锂电典型放电曲线 (3.00V 空 -> 4.20V 满)，中段 3.7-4.0V
 * 平台区较平缓，两端陡降，用单条两点直线无法准确表达。
 * ========================================================================== */

/** OCV -> SOC 标定点 */
typedef struct {
    uint16_t mv;    /*!< 开路电压 (mV) */
    uint8_t  soc;   /*!< 电量 (%) */
} soc_point_t;

static const soc_point_t SOC_CURVE[] = {
    { 3000,   0 }, { 3300,   3 }, { 3500,   6 }, { 3610,  10 },
    { 3650,  13 }, { 3690,  17 }, { 3710,  20 }, { 3730,  25 },
    { 3750,  30 }, { 3770,  35 }, { 3790,  40 }, { 3800,  45 },
    { 3820,  50 }, { 3840,  55 }, { 3850,  60 }, { 3870,  65 },
    { 3910,  70 }, { 3950,  75 }, { 3980,  80 }, { 4020,  85 },
    { 4080,  90 }, { 4110,  95 }, { 4200, 100 },
};

#define SOC_CURVE_LEN   (sizeof(SOC_CURVE) / sizeof(SOC_CURVE[0]))

/**
 * @brief 由开路电压查表估算电量
 *
 * 标定点之间线性插值。
 *
 * @param mv 开路电压 (mV)
 * @return 电量百分比 0-100
 */
static uint8_t soc_from_voltage(uint16_t mv)
{
    if (mv <= SOC_CURVE[0].mv) {
        return SOC_CURVE[0].soc;
    }
    if (mv >= SOC_CURVE[SOC_CURVE_LEN - 1].mv) {
        return SOC_CURVE[SOC_CURVE_LEN - 1].soc;
    }

    for (size_t i = 1; i < SOC_CURVE_LEN; i++) {
        if (mv <= SOC_CURVE[i].mv) {
            const soc_point_t *lo = &SOC_CURVE[i - 1];
            const soc_point_t *hi = &SOC_CURVE[i];
            uint32_t span = (uint32_t)(hi->mv - lo->mv);
            uint32_t num  = (uint32_t)(mv - lo->mv) *
                            (uint32_t)(hi->soc - lo->soc);
            return (uint8_t)(lo->soc + num / span);
        }
    }
    return 100;     /* 不可达 */
}

/* ============================================================================
 * 引脚复用: 从 USB Serial/JTAG 释放 IO12/IO13
 * ========================================================================== */

/**
 * @brief 把 IO12/IO13 从 USB 外设释放，准备用作 I2C
 *
 * ESP32-C6 的 IO12/IO13 默认复用为 USB Serial/JTAG 的 D-/D+。
 * IAP 程序初始化过 USB 串口，因此这里必须按顺序做三件事:
 *
 *   1. 卸载 USB 串口驱动 (释放中断与环形缓冲)
 *   2. 关闭 USB PHY pad 与模块时钟
 *      —— 这一步最关键。IDF 的 usb_serial_jtag_driver_uninstall() 源码里
 *         明确注释「不在这里关闭模块时钟与 usb_pad_enable，因为 stdout
 *         可能仍依赖它」，所以仅卸载驱动**不会**释放 IO12/IO13，
 *         必须显式调用 LL 层函数。
 *   3. 复位两个引脚为普通 GPIO，清除 ROM/IAP 遗留的配置
 *
 * ⚠️ 副作用: 释放后 USB 串口终端失效。本应用通过 WiFi/BLE 提供控制
 *    通道，日志仍可从 UART0 输出，因此不受影响。
 */
static void release_usb_pins(void)
{
    /*
     * 1. 卸载 USB 串口驱动。
     *
     * 返回 ESP_OK 也可能是「本就没安装」(IDF 实现如此)，两种情况都正常。
     */
    esp_err_t err = usb_serial_jtag_driver_uninstall();
    ESP_LOGD(TAG, "usb_serial_jtag_driver_uninstall: %s",
             esp_err_to_name(err));

    /*
     * 2. 关闭 USB PHY pad 与模块时钟，真正释放 IO12/IO13。
     *
     * 先关 pad 再关时钟 —— 顺序与 IDF 内部启用时相反，确保 pad 在
     * 时钟关闭前已停止驱动引脚。
     */
    usb_serial_jtag_ll_phy_enable_pad(false);
    usb_serial_jtag_ll_enable_bus_clock(false);

    /*
     * 3. 复位引脚。
     *
     * gpio_reset_pin() 会断开引脚的 IO MUX 输出/输入通道并清除上下拉，
     * 使引脚回到干净的复位态，之后 I2C 主机初始化才能正确接管。
     */
    gpio_reset_pin((gpio_num_t)BATTERY_IP5108_SCL_GPIO);
    gpio_reset_pin((gpio_num_t)BATTERY_IP5108_SDA_GPIO);

    ESP_LOGI(TAG, "IO%d(SCL)/IO%d(SDA) 已从 USB 释放并复位",
             BATTERY_IP5108_SCL_GPIO, BATTERY_IP5108_SDA_GPIO);
}

/**
 * @brief 等待 IP5108 进入 I2C 模式
 *
 * 若配置了 INT 引脚，则等待其拉高（表示芯片已确认 I2C 模式）。
 * 未配置时直接返回 true（尽力而为）。
 *
 * @return true 可以访问总线
 */
static bool wait_i2c_ready(void)
{
#if BATTERY_IP5108_INT_GPIO >= 0
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BATTERY_IP5108_INT_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    int64_t deadline = esp_timer_get_time() +
                       (int64_t)BATTERY_IP5108_INT_TIMEOUT * 1000;

    while (gpio_get_level((gpio_num_t)BATTERY_IP5108_INT_GPIO) == 0) {
        if (esp_timer_get_time() >= deadline) {
            ESP_LOGW(TAG, "等待 INT 就绪超时 (%d ms)",
                     BATTERY_IP5108_INT_TIMEOUT);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGI(TAG, "IP5108 已进入 I2C 模式 (INT 拉高)");
    return true;
#else
    /* 未接 INT 引脚: 无法确认，直接尝试探测总线 */
    return true;
#endif
}

/* ============================================================================
 * 初始化
 * ========================================================================== */

esp_err_t battery_ip5108_init(void)
{
    if (s_bus != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* --- 1. 释放 IO12/IO13 --- */
    release_usb_pins();

    /* --- 2. 等待芯片就绪 --- */
    wait_i2c_ready();

    /* --- 3. 创建 I2C 主机总线 --- */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = BATTERY_IP5108_SDA_GPIO,
        .scl_io_num        = BATTERY_IP5108_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /*
         * 启用内部上拉。
         *
         * IP5108 的 L1/L2 在 I2C 模式下由芯片自己上拉到 VREG，
         * 但探测阶段（芯片可能尚未进入 I2C 模式）总线可能是浮空的，
         * 开内部上拉可避免读到随机值。
         */
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建 I2C 总线失败: %s", esp_err_to_name(err));
        s_bus = NULL;
        return err;
    }

    /* --- 4. 挂载 IP5108 从机 --- */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BATTERY_IP5108_I2C_ADDR,
        .scl_speed_hz    = BATTERY_IP5108_I2C_FREQ,
    };

    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "挂载 IP5108 失败: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        s_dev = NULL;
        return err;
    }

    /* --- 5. 探测芯片 --- */
    err = i2c_master_probe(s_bus, BATTERY_IP5108_I2C_ADDR, 200);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "未探测到 IP5108 (地址 0x%02X): %s",
                 BATTERY_IP5108_I2C_ADDR, esp_err_to_name(err));
        ESP_LOGW(TAG, "电池信息将不可用 (可能未接电池或芯片处于 LED 模式)");
        /*
         * 不释放总线 —— 保留句柄，允许后续重试读取。
         * s_ready 保持 false，battery_ip5108_available() 会返回 false。
         */
        return ESP_ERR_NOT_FOUND;
    }

    s_ready = true;
    ESP_LOGI(TAG, "IP5108 电量计就绪 (I2C 0x%02X, SCL=IO%d, SDA=IO%d)",
             BATTERY_IP5108_I2C_ADDR,
             BATTERY_IP5108_SCL_GPIO, BATTERY_IP5108_SDA_GPIO);

    /* --- 6. 读一次确认数据有效 --- */
    battery_status_t st;
    if (battery_ip5108_read(&st) == ESP_OK) {
        ESP_LOGI(TAG, "电池: %.3fV %.0fmA %d%% %s",
                 st.voltage, st.current * 1000.0f, st.percent,
                 battery_charge_status_desc(st.charge_status));
    }

    return ESP_OK;
}

bool battery_ip5108_available(void)
{
    return s_ready && s_dev != NULL;
}

/* ============================================================================
 * 读取状态
 * ========================================================================== */

esp_err_t battery_ip5108_read(battery_status_t *st)
{
    if (st == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(st, 0, sizeof(*st));
    st->percent = -1;

    /* --- 电压 / 电流 / 开路电压 --- */
    uint16_t raw;

    if (adc_read_pair(REG_BATVADC_DAT0, &raw) == ESP_OK) {
        st->voltage = decode_voltage(raw);
    } else {
        ESP_LOGW(TAG, "读取电池电压失败");
    }

    if (adc_read_pair(REG_BATIADC_DAT0, &raw) == ESP_OK) {
        st->current = decode_current(raw);
    } else {
        ESP_LOGW(TAG, "读取电池电流失败");
    }

    bool ocv_ok = (adc_read_pair(REG_BATOCV_DAT0, &raw) == ESP_OK);
    if (ocv_ok) {
        st->ocv = decode_voltage(raw);
    } else {
        ESP_LOGW(TAG, "读取开路电压失败");
    }

    /* --- 充电状态 (0x71) --- */
    uint8_t r71 = 0;
    if (reg_read(REG_READ0B, &r71) == ESP_OK) {
        st->charge_status   = (r71 >> 5) & 0x07;
        st->charging        = (st->charge_status == BATTERY_CHARGE_TRICKLE ||
                               st->charge_status == BATTERY_CHARGE_CC ||
                               st->charge_status == BATTERY_CHARGE_CV);
        st->charge_done     = (r71 >> 3) & 0x01;
        st->cv_timeout      = (r71 >> 2) & 0x01;
        st->charge_timeout  = (r71 >> 1) & 0x01;
        st->trickle_timeout = (r71 >> 0) & 0x01;
    } else {
        ESP_LOGW(TAG, "读取充电状态失败");
    }

    /* --- 负载 / 输入状态 (0x72) --- */
    uint8_t r72 = 0;
    if (reg_read(REG_READ1, &r72) == ESP_OK) {
        st->load_connected    = (r72 >> 7) & 0x01;
        st->light_load        = (r72 >> 6) & 0x01;
        st->input_overvoltage = (r72 >> 5) & 0x01;
    }

    /* --- 按键状态 (0x77) --- */
    uint8_t r77 = 0;
    if (reg_read(REG_READ2, &r77) == ESP_OK) {
        st->button_pressed     = (r77 >> 3) & 0x01;
        st->button_long_press  = (r77 >> 1) & 0x01;
        st->button_short_press = (r77 >> 0) & 0x01;
    }

    /* --- 电量百分比 --- */
    if (st->charge_status == BATTERY_CHARGE_CV_STOP ||
        st->charge_status == BATTERY_CHARGE_FULL) {
        /* 硬件确认已充满，直接 100% */
        st->percent = 100;
    } else if (ocv_ok && st->ocv > 0.0f) {
        st->percent = (int8_t)soc_from_voltage((uint16_t)(st->ocv * 1000.0f + 0.5f));
    }

    return ESP_OK;
}

/* ============================================================================
 * 状态名转换
 * ========================================================================== */

const char *battery_charge_status_name(uint8_t status)
{
    switch (status) {
    case BATTERY_CHARGE_IDLE:      return "idle";
    case BATTERY_CHARGE_TRICKLE:   return "trickle";
    case BATTERY_CHARGE_CC:        return "cc";
    case BATTERY_CHARGE_CV:        return "cv";
    case BATTERY_CHARGE_CV_STOP:   return "cv_stop";
    case BATTERY_CHARGE_FULL:      return "full";
    case BATTERY_CHARGE_TIMEOUT:   return "timeout";
    default:                       return "unknown";
    }
}

const char *battery_charge_status_desc(uint8_t status)
{
    switch (status) {
    case BATTERY_CHARGE_IDLE:      return "未充电";
    case BATTERY_CHARGE_TRICKLE:   return "涓流充电";
    case BATTERY_CHARGE_CC:        return "恒流充电";
    case BATTERY_CHARGE_CV:        return "恒压充电";
    case BATTERY_CHARGE_CV_STOP:   return "恒压结束";
    case BATTERY_CHARGE_FULL:      return "已充满";
    case BATTERY_CHARGE_TIMEOUT:   return "充电超时";
    default:                       return "未知";
    }
}
