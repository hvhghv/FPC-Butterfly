/*
 * 蝴蝶板灯控应用 — IP5108 移动电源 SoC 电量计驱动
 *
 * 【硬件连接】
 *
 *   IP5108 的 I2C 端口接到 ESP32-C6 的:
 *     SCL -> IO12
 *     SDA -> IO13
 *
 *   ⚠️ IO12/IO13 在 ESP32-C6 上是 USB Serial/JTAG 的 D-/D+ 引脚，
 *      IAP 程序会把它们初始化为 USB 串口（终端 + 烧录）。
 *      本用户程序**不使用 USB**，因此在初始化 I2C 前必须先把这两个引脚
 *      从 USB 外设上"拿回来"（见 battery_ip5108.c 的引脚复用说明）。
 *
 * 【IP5108 简介】
 *
 *   Injoinic（英集芯）的移动电源 SoC，集成:
 *     - 1.2A 锂电池充电管理
 *     - 1.0A 升压转换器
 *     - 14-bit ADC（精确测量电池电压与电流）
 *     - 内置电量计算法
 *
 *   通过 I2C 可读取电池电压 / 电流 / 开路电压 / 充电状态 / 电量百分比。
 *
 * 【I2C 协议】
 *
 *   从机地址: 7-bit 0x75（写 0xEA / 读 0xEB）
 *   速率:     400 kHz
 *   时序:     标准 I2C 寄存器读写（先写寄存器地址，再读数据）
 *
 * 【关键寄存器】
 *
 *   只读状态:
 *     0x71 REG_READ0B  充电状态 + 各阶段超时标志
 *     0x72 REG_READ1   负载/输入状态
 *     0x77 REG_READ2   按键状态
 *
 *   ADC 数据（各 2 字节，低字节在前）:
 *     0xA2/0xA3  BATVADC  电池电压（充电侧，含 IR 补偿）
 *     0xA4/0xA5  BATIADC  电池电流（充电为正，放电为负）
 *     0xA8/0xA9  BATOCV   开路电压 OCV（推荐用于估算电量）
 *
 * 【⚠️ INT 引脚注意事项】
 *
 *   IP5108 在每次 sleep -> wake 时都会重新采样 L1/L2 (SCL/SDA) 是否被
 *   上拉到 VREG，以此决定进入 I2C 模式还是 LED 指示模式。这个决策一旦
 *   做出，在本次供电周期内不会再改变。
 *
 *   若 MCU 在该采样完成前就开始访问总线（哪怕是一次正常的 I2C 事务），
 *   可能干扰采样，导致芯片退回 LED 模式。
 *
 *   数据手册建议: MCU 保持 SDA/SCL 高阻，直到 INT 引脚（L3）拉高。
 *   本驱动在初始化时等待 INT 就绪，见 BATTERY_IP5108_INT_GPIO。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 硬件配置
 * ========================================================================== */

/** IP5108 I2C 从机地址 (7-bit) */
#define BATTERY_IP5108_I2C_ADDR     0x75

/**
 * I2C 默认时钟频率 (100 kHz)。
 *
 * ⚠️ 默认用**低速**启动。原因:
 *   - IP5108 数据手册要求 SCL/SDA 外接上拉到 VREG，实际板子上的上拉
 *     阻值/走线电容往往不理想，400kHz 下上升沿过慢会导致通信失败。
 *   - 100kHz 对波形容忍度高得多，能显著提高首次连接成功率。
 *   - 电池信息刷新频率很低 (秒级)，低速完全不影响体验。
 *
 * 若探测成功且总线条件良好，可参考 BATTERY_IP5108_I2C_FREQ_FAST 提速。
 */
#define BATTERY_IP5108_I2C_FREQ     100000

/**
 * 高速档位 (400 kHz, IP5108 支持的上限)。
 *
 * 仅在低速探测成功后，由用户显式切换或诊断确认波形良好时使用。
 */
#define BATTERY_IP5108_I2C_FREQ_FAST    400000

/** I2C SCL 引脚 (与 IP5108 的 L1 相连) */
#define BATTERY_IP5108_SCL_GPIO     12

/** I2C SDA 引脚 (与 IP5108 的 L2 相连) */
#define BATTERY_IP5108_SDA_GPIO     13

/**
 * IP5108 的 INT (L3) 引脚所接的 GPIO。
 *
 * 用于在访问 I2C 前确认芯片已进入 I2C 模式。若硬件未连接该引脚，
 * 定义为 -1 表示不使用（此时初始化直接尝试探测总线）。
 */
#define BATTERY_IP5108_INT_GPIO     (-1)

/** 等待 INT 就绪的超时 (毫秒) */
#define BATTERY_IP5108_INT_TIMEOUT  2000

/**
 * 是否启用内部上拉。
 *
 * ⚠️ IP5108 数据手册要求 SCL/SDA **外接上拉到 VREG**。
 *    ESP32 内部上拉约 45kΩ，在 400kHz 下上升沿过慢会导致波形畸变，
 *    因此硬件上**必须**有外部上拉电阻 (典型 2.2k - 10k)。
 *
 *    这里保留内部上拉仅作为"兜底"：当外部上拉缺失或阻值偏大时，
 *    配合降低时钟频率 (见 BATTERY_IP5108_I2C_FREQ_FALLBACK) 仍可能通信。
 *    若你的板子已正确外接上拉，可置 0 以减少总线负载。
 */
#define BATTERY_IP5108_USE_INTERNAL_PULLUP  1

/**
 * 初始化失败时的降级速率。
 *
 * 默认已用 100kHz，若仍探测失败则降到 50kHz 再试一次。
 * 更低速率对上升沿要求更宽松，可用于区分
 * "上拉不足" 与 "芯片未进入 I2C 模式" 两类故障。
 */
#define BATTERY_IP5108_I2C_FREQ_FALLBACK    50000

/* ============================================================================
 * 诊断
 * ========================================================================== */

/**
 * @brief 总线诊断结果
 */
typedef struct {
    bool     scl_high;            /*!< SCL 空闲时是否为高电平 */
    bool     sda_high;            /*!< SDA 空闲时是否为高电平 */
    uint8_t  found_addr;          /*!< 扫描到的第一个应答地址，0 表示无 */
    uint8_t  scan_count;          /*!< 应答设备总数 */
    bool     probe_400k;          /*!< 400kHz 下是否探测到 0x75 */
    bool     probe_100k;          /*!< 100kHz 下是否探测到 0x75 */
    bool     probe_50k;           /*!< 50kHz 下是否探测到 0x75 */
    int      int_level;           /*!< INT 引脚电平，-1 表示未配置 */
} battery_diag_t;

/**
 * @brief 诊断 I2C 总线（排查"没反应"问题）
 *
 * 依次执行:
 *   1. 把 SCL/SDA 配为输入，读取空闲电平
 *      —— 两者都应为高（被上拉）。若为低，说明上拉缺失或总线被拉死。
 *   2. 扫描 0x08-0x77 全部地址，列出所有应答设备
 *      —— 若扫到别的地址，说明接线或地址有误。
 *   3. 分别在 400kHz / 100kHz 下探测 0x75
 *      —— 若仅 100kHz 成功，说明上拉不足（内部上拉太弱）。
 *
 * 本函数会临时重建 I2C 总线，因此应在 battery_ip5108_init() 之前调用，
 * 或在初始化失败后调用。
 *
 * @param[out] diag 输出诊断结果
 * @return ESP_OK 诊断已执行（结果需自行判读）
 */
esp_err_t battery_ip5108_diag(battery_diag_t *diag);

/* ============================================================================
 * 充电状态
 * ========================================================================== */

/**
 * @brief IP5108 充电状态 (寄存器 0x71 的 bit7-5)
 */
typedef enum {
    BATTERY_CHARGE_IDLE = 0,      /*!< 未充电 */
    BATTERY_CHARGE_TRICKLE,       /*!< 涓流充电 */
    BATTERY_CHARGE_CC,            /*!< 恒流充电 */
    BATTERY_CHARGE_CV,            /*!< 恒压充电 */
    BATTERY_CHARGE_CV_STOP,       /*!< 恒压结束检测 */
    BATTERY_CHARGE_FULL,          /*!< 充电完成 */
    BATTERY_CHARGE_TIMEOUT,       /*!< 充电超时 */
    BATTERY_CHARGE_STATUS_MAX,
} battery_charge_status_t;

/* ============================================================================
 * 数据结构
 * ========================================================================== */

/**
 * @brief 电池状态快照
 *
 * 由 battery_ip5108_read() 一次性读取并解码，避免多次 I2C 往返。
 */
typedef struct {
    /* --- 测量值 --- */
    float    voltage;             /*!< 电池电压 (V)，来自 BATVADC */
    float    ocv;                 /*!< 开路电压 (V)，来自 BATOCV */
    float    current;             /*!< 电池电流 (A)，充电为正 / 放电为负 */

    /* --- 电量 --- */
    int8_t   percent;             /*!< 电量百分比 0-100，-1 表示无效 */

    /* --- 充电状态 --- */
    uint8_t  charge_status;       /*!< battery_charge_status_t */
    bool     charging;            /*!< 是否正在充电 (涓流/CC/CV) */
    bool     charge_done;         /*!< 充电是否结束 */
    bool     charge_timeout;      /*!< 充电超时 */
    bool     trickle_timeout;     /*!< 涓流充电超时 */
    bool     cv_timeout;          /*!< 恒压阶段超时 */

    /* --- 输入 / 负载状态 (寄存器 0x72) --- */
    bool     load_connected;      /*!< 是否检测到负载 */
    bool     light_load;          /*!< 轻载标志 */
    bool     input_overvoltage;   /*!< 输入过压 */

    /* --- 按键状态 (寄存器 0x77) --- */
    bool     button_pressed;      /*!< 按键当前按下 */
    bool     button_long_press;   /*!< 长按事件 */
    bool     button_short_press;  /*!< 短按事件 */
} battery_status_t;

/* ============================================================================
 * 接口
 * ========================================================================== */

/**
 * @brief 初始化 IP5108 电量计
 *
 * 完成以下工作:
 *   1. 把 IO12/IO13 从 USB Serial/JTAG 外设释放，重新配置为 I2C 引脚
 *   2. (若配置了 INT 引脚) 等待 IP5108 进入 I2C 模式
 *   3. 创建 I2C 主机总线并探测从机
 *
 * 若探测失败（芯片不存在或未进入 I2C 模式），返回错误但**不阻塞**
 * 应用启动 —— 上层应把电池信息标记为不可用。
 *
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_STATE 已初始化
 *         ESP_ERR_NOT_FOUND 未探测到 IP5108
 *         其他 I2C 错误
 */
esp_err_t battery_ip5108_init(void);

/**
 * @brief 重新初始化 I2C 总线并重新探测芯片
 *
 * 用于"刷新"场景: 设备运行中电池被拔出/插入、或 IP5108 经历一次
 * sleep->wake 重新进入 I2C 模式后，调用本函数可重建总线并重新探测，
 * 无需重启整个设备。
 *
 * 内部流程: 销毁旧总线 → 释放引脚 → 重建总线 → 探测 0x75 (含降速重试)。
 * 若已初始化则先释放旧资源，因此可安全重复调用。
 *
 * @return ESP_OK 重新探测成功
 *         ESP_ERR_NOT_FOUND 仍未探测到 IP5108
 *         其他 I2C 错误
 */
esp_err_t battery_ip5108_reinit(void);

/**
 * @brief 读取一次电池状态
 *
 * 读取电压/电流/OCV/充电状态/输入状态/按键状态，并计算电量百分比。
 *
 * @param[out] st 输出状态快照
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG st 为 NULL
 *         ESP_ERR_INVALID_STATE 未初始化
 *         其他 I2C 错误
 */
esp_err_t battery_ip5108_read(battery_status_t *st);

/**
 * @brief 查询电量计是否可用
 *
 * @return true 已初始化且探测到芯片
 */
bool battery_ip5108_available(void);

/**
 * @brief 把充电状态枚举转成可读字符串
 *
 * @param status battery_charge_status_t
 * @return 英文标识串（用于 JSON），如 "cc" / "full"
 */
const char *battery_charge_status_name(uint8_t status);

/**
 * @brief 把充电状态枚举转成中文描述
 *
 * @param status battery_charge_status_t
 * @return 中文描述，如 "恒流充电" / "已充满"
 */
const char *battery_charge_status_desc(uint8_t status);

#ifdef __cplusplus
}
#endif
