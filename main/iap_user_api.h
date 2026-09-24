/*
 * ESP IAP - 用户程序侧集成接口
 *
 * 用户程序通过本接口与 IAP 程序交互:
 *   - 读取 IAP 配置 (经 RTC RAM 拿地址，再直读 flash)
 *   - 请求重启进入 IAP 下载模式
 *   - 上报自身运行状态
 *
 * ============================================================================
 *  v5 架构要点 (详见 docs/FINAL-REPORT.md)
 * ============================================================================
 *
 *  1. **独立分区表**
 *     用户程序使用**分区表 B** (位于 0x140000)，与 IAP 的分区表 A
 *     (0xB000) 完全独立、互不重叠。用户程序**看不到** IAP 的分区。
 *
 *  2. **iap_cfg 是硬编码共享区**
 *     iap_cfg (0x008000, 8KB) **不在任何分区表**中，
 *     因此**不能**用 esp_partition_find_first() 查找。
 *     用户程序通过 RTC RAM 得到 cfg_addr，再用 esp_flash_read() 直读。
 *
 *  3. **RTC RAM 传递启动参数**
 *     IAP 在启动用户程序前把 iap_boot_param_t 写入 RTC RAM
 *     (rtc_retain_mem_t.custom[])，用户程序启动后读取。
 *     详见 iap_user_boot_param.h
 *
 *  4. **回 IAP 的方式**
 *     写 RTC RAM boot_target=0 → esp_restart()
 *     (不再使用 esp_ota_set_boot_partition)
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
 * 与 IAP 端保持一致的定义
 * ========================================================================== */

/*
 * ⚠️ v5: **配置区 (0x8000) 对用户程序不开放**。
 *
 * 用户程序**不得**读写配置区:
 *   - 配置区在 IDF flash 写保护区 (0x0 ~ 0xBFFF) 内，直接写会 abort()
 *   - 配置区是 IAP 私有数据，用户程序读写会破坏双槽一致性
 *
 * 需要配置时: 通过 RTC RAM 启动参数只读获取，或用
 * iap_user_request_download() 请求重启进 IAP 修改。
 */

/** 用户程序区地址 */
#define IAP_USER_APP_ADDR                  0x150000

/** IAP 程序区地址 */
#define IAP_USER_IAP_APP_ADDR              0x010000

/** 配置区布局 (仅供理解协议，用户程序**不可**直接访问) */
#define IAP_USER_CFG_MAGIC                 0x49415043U  /*!< "IAPC" */
#define IAP_USER_CFG_VERSION               0x0001
#define IAP_USER_CFG_SLOT_SIZE             4096    /*!< 必须与 IAP 端 IAP_CFG_SLOT_SIZE 一致 */
#define IAP_USER_CFG_SLOT_COUNT            2

/** 配置标志位 */
#define IAP_USER_CFG_FLAG_DOWNLOAD_MODE    (1U << 0)  /*!< 置位则进入 IAP 下载模式 */
#define IAP_USER_CFG_FLAG_WIFI_ENABLE      (1U << 1)
#define IAP_USER_CFG_FLAG_I2C_ENABLE       (1U << 2)
#define IAP_USER_CFG_FLAG_UART_ENABLE      (1U << 3)
#define IAP_USER_CFG_FLAG_VERIFY_USER_APP  (1U << 4)

/** 等待期间进入 IAP 下载模式的触发源开关 */
#define IAP_USER_CFG_FLAG_WAIT_GPIO_TRIG   (1U << 5)  /*!< GPIO 引脚电平触发 */
#define IAP_USER_CFG_FLAG_WAIT_I2C_TRIG    (1U << 6)  /*!< I2C 进入命令触发 */
#define IAP_USER_CFG_FLAG_WAIT_UART_TRIG   (1U << 7)  /*!< UART 进入命令触发 */
#define IAP_USER_CFG_FLAG_WAIT_WIFI_TRIG   (1U << 8)  /*!< WiFi 客户端接入触发 */

/** USB 相关标志位 */
#define IAP_USER_CFG_FLAG_USB_ENABLE       (1U << 9)  /*!< 使能 USB 串口 (终端 + 烧录) */
#define IAP_USER_CFG_FLAG_WAIT_USB_TRIG    (1U << 10) /*!< USB 进入命令触发 */

/** 触发引脚有效电平 */
#define IAP_USER_GPIO_TRIG_ACTIVE_LOW      0         /*!< 低电平有效 (内部上拉) */
#define IAP_USER_GPIO_TRIG_ACTIVE_HIGH     1         /*!< 高电平有效 (内部下拉) */

/** 触发引脚未配置 */
#define IAP_USER_DEFAULT_TRIG_GPIO         0xFF

/** 启动参数字符串标志位 (仅用于理解 IAP 侧协议) */
#define IAP_USER_BOOT_PARAM_FLAG_VALID     (1U << 0)  /*!< 启动参数有效 */
#define IAP_USER_BOOT_PARAM_FLAG_CONSUMED  (1U << 1)  /*!< 已被用户程序读取 */

/**
 * 启动参数字符串缓冲区建议长度 (含结尾 '\0')。
 *
 * v5: 实际参数字符串经 RTC RAM 传递，最大 IAP_USER_PARAM_STR_SIZE 字节。
 *     调用 iap_user_get_boot_param() 时缓冲区可传本值 (256) 以避免溢出。
 */
#define IAP_USER_BOOT_PARAM_SIZE           256

/** 启动参数字符串最大可写字符数 (不含结尾 '\0') */
#define IAP_USER_BOOT_PARAM_MAX_LEN        (IAP_USER_BOOT_PARAM_SIZE - 1)

/** 启动原因 */
#define IAP_USER_BOOT_REASON_NONE          0
#define IAP_USER_BOOT_REASON_DOWNLOAD_FLAG 1
#define IAP_USER_BOOT_REASON_WAIT_TIMEOUT  2
#define IAP_USER_BOOT_REASON_USER_REQUEST  3
#define IAP_USER_BOOT_REASON_CRC_FAILED    4
#define IAP_USER_BOOT_REASON_NO_VALID_APP  5
#define IAP_USER_BOOT_REASON_FIRST_BOOT    6

/*
 * ⚠️ v5: 用户程序**不定义**配置数据结构。
 *
 * 配置区 (0x8000) 是 IAP 的私有数据，用户程序不可访问。
 * 早期版本在此定义了 iap_user_cfg_header_t / iap_user_cfg_t，
 * 现全部移除，避免用户程序误用。
 *
 * 用户程序需要的信息通过 RTC RAM 启动参数获取 (见文件末尾)。
 */

/* ============================================================================
 * 接口
 * ========================================================================== */

/*
 * ⚠️ v5: **用户程序不允许访问配置区**。
 *
 * 原因:
 *   1. 配置区 (0x8000) 位于 IDF 的 flash 写保护区 (0x0 ~ 0xBFFF)，
 *      用户程序直接写会触发 abort()。
 *   2. 配置区是 IAP 的私有数据，用户程序读写会破坏双槽冗余的一致性。
 *
 * 用户程序获取配置的方式:
 *   - **只读**: 通过 RTC RAM 启动参数 (iap_user_boot_param_read)
 *     或启动参数字符串 (iap_user_get_boot_param)
 *   - **请求修改**: 调用 iap_user_request_download() 重启进 IAP，
 *     由用户在 IAP 终端 / HTML 工具中修改
 */

/**
 * @brief 请求重启进入 IAP 下载模式
 *
 * 写 RTC RAM 启动参数 (boot_target=IAP) 后重启。
 * IAP 启动后会进入下载模式，用户可在此修改配置。
 *
 * @return 仅在失败时返回；成功时不会返回 (已重启)
 */
esp_err_t iap_user_request_download(void);

/**
 * @brief 检查是否处于 IAP 下载模式
 *
 * @return true 表示 IAP 处于下载模式 (由 RTC RAM 判定)
 */
bool iap_user_is_download_requested(void);

/* ============================================================================
 * 启动参数字符串
 *
 * IAP 在启动用户程序前写入启动参数，用户程序启动后读取以完成初始化。
 * 参数格式为 "key1=value1;key2=value2"，由 IAP 与用户程序自行约定。
 * ========================================================================== */

/**
 * @brief 读取启动参数字符串
 *
 * 会校验 CRC32。典型用法:
 *
 *   char param[IAP_USER_BOOT_PARAM_SIZE];
 *   if (iap_user_get_boot_param(param, sizeof(param)) == ESP_OK) {
 *       char mode[16];
 *       if (iap_user_boot_param_get_value(param, "mode", mode, sizeof(mode)) == ESP_OK) {
 *           // 根据 mode 初始化
 *       }
 *   }
 *
 * @param[out] buf     输出缓冲区
 * @param[in]  buf_len 缓冲区长度，建议 IAP_USER_BOOT_PARAM_SIZE
 * @return ESP_OK 成功
 *         ESP_ERR_NOT_FOUND 启动参数未设置
 *         ESP_ERR_INVALID_SIZE 缓冲区不足
 *         ESP_ERR_INVALID_CRC CRC 校验失败
 */
esp_err_t iap_user_get_boot_param(char *buf, size_t buf_len);

/**
 * @brief 读取启动参数并标记为已消费
 *
 * 便于实现"仅首次启动生效"的参数。
 *
 * @param[out] buf     输出缓冲区
 * @param[in]  buf_len 缓冲区长度
 * @return ESP_OK 成功
 */
esp_err_t iap_user_take_boot_param(char *buf, size_t buf_len);

/**
 * @brief 从启动参数中提取指定键的值
 *
 * @param[in]  param   参数字符串
 * @param[in]  key     键名
 * @param[out] out     输出值缓冲区
 * @param[in]  out_len 缓冲区长度
 * @return ESP_OK 成功
 *         ESP_ERR_NOT_FOUND 未找到该键
 *         ESP_ERR_INVALID_SIZE 缓冲区不足
 */
esp_err_t iap_user_boot_param_get_value(const char *param, const char *key,
                                        char *out, size_t out_len);

/**
 * @brief 查询启动参数是否有效
 *
 * @return true 表示已由 IAP 写入且 CRC 正确
 */
bool iap_user_has_boot_param(void);

/**
 * @brief 查询启动参数是否已被消费
 *
 * @return true 表示已消费
 */
bool iap_user_boot_param_consumed(void);

/* ============================================================================
 * RTC RAM 启动参数 (v5)
 *
 * IAP 在启动用户程序前把 iap_boot_param_t 写入 RTC RAM:
 *   rtc_retain_mem_t *mem = bootloader_common_get_rtc_retain_mem();
 *   iap_boot_param_t *p = (iap_boot_param_t *)mem->custom;
 *
 * 用户程序可据此得知:
 *   - 启动目标 (应总是 APP)
 *   - 自身加载地址/大小
 *   - 启动参数字符串 (param)
 * ========================================================================== */

/** 启动参数魔术字 ("IAPB") */
#define IAP_USER_PARAM_MAGIC               0x49415042U

/** 启动目标: IAP */
#define IAP_USER_BOOT_TARGET_IAP           0

/** 启动目标: 用户程序 */
#define IAP_USER_BOOT_TARGET_APP           1

/** 启动参数字符串最大长度 (与 IAP 端 IAP_PARAM_STR_SIZE 一致) */
#define IAP_USER_PARAM_STR_SIZE            32

/** 预留区大小 (与 IAP 端 IAP_PARAM_RESERVED_SIZE 一致) */
#define IAP_USER_PARAM_RESERVED_SIZE       8

/* -------------------------------------------------------------------------- */
/* 用户程序可请求更新的字段 (v6)                                                */
/* -------------------------------------------------------------------------- */
/*
 * 用户程序**不能直接访问配置区**。需要持久化修改时:
 *   1. 调用 iap_user_request_*() 填写 RTC RAM update 区
 *   2. 调用 iap_user_request_commit() 重算 CRC
 *   3. 调用 iap_user_request_apply() 重启进 IAP
 *   4. IAP 检测到请求 → 写入配置区 → 重启启动用户程序
 *
 * 只能修改 IAP 暴露的白名单字段 (见下)。
 */

/** 请求修改启动参数字符串 */
#define IAP_USER_UPD_BOOT_PARAM            (1U << 0)
/** 请求修改 OTA 槽序号 */
#define IAP_USER_UPD_ACTIVE_SLOT           (1U << 1)
/** 请求修改加载地址 */
#define IAP_USER_UPD_USER_ADDR             (1U << 2)
/** 请求修改启动目标 */
#define IAP_USER_UPD_BOOT_TARGET           (1U << 3)

/** 允许修改的字段掩码 (与 IAP 端 IAP_PARAM_UPD_ALLOWED_MASK 一致) */
#define IAP_USER_UPD_ALLOWED_MASK          (IAP_USER_UPD_BOOT_PARAM  | \
                                            IAP_USER_UPD_ACTIVE_SLOT | \
                                            IAP_USER_UPD_USER_ADDR   | \
                                            IAP_USER_UPD_BOOT_TARGET)

/**
 * @brief 启动参数结构 (与 IAP 端 iap_boot_param_t 逐字节一致)
 *
 * v5: **只传启动参数** —— 配置区对用户程序不开放，因此
 *     启动参数字符串 (param) 也通过本结构经 RTC RAM 传递。
 * v6: 新增 **update 区** —— 用户程序把要持久化的修改写在这里，
 *     重启进 IAP 后由 IAP 写入配置区。
 */
typedef struct {
    uint32_t magic;          /*!< IAP_USER_PARAM_MAGIC */
    uint32_t version;        /*!< 结构版本 */
    uint32_t crc32;          /*!< 以下数据的 CRC32 */

    uint8_t  boot_target;    /*!< 0 = IAP, 1 = APP */
    uint8_t  reserved0[3];
    uint32_t user_addr;      /*!< 用户程序加载地址 (0x150000) */
    uint32_t user_size;      /*!< 用户程序大小 */
    uint32_t user_version;   /*!< 用户程序版本号 */

    uint32_t cfg_addr;       /*!< iap_cfg 地址 (仅供 IAP 内部使用) */
    uint32_t cfg_seq;        /*!< 配置槽序号 */

    uint8_t  param_len;      /*!< 启动参数字符串有效字符数 */
    uint8_t  param_reserved[3];
    char     param[IAP_USER_PARAM_STR_SIZE]; /*!< "key1=value1;key2=value2" */

    /* --- 用户程序请求更新区 (v6) --- */
    uint8_t  update_flags;   /*!< IAP_USER_UPD_* 位掩码, 0 = 无请求 */
    uint8_t  upd_active_slot;/*!< 请求的 OTA 槽序号 */
    uint8_t  upd_reserved[2];
    uint32_t upd_user_addr;  /*!< 请求的加载地址 */
    uint8_t  upd_boot_target;/*!< 请求的启动目标 */
    uint8_t  upd_param_len;  /*!< 请求的参数字符串长度 */
    uint8_t  upd_param_reserved[2];
    char     upd_param[IAP_USER_PARAM_STR_SIZE]; /*!< 请求的参数字符串 */

    uint8_t  reserved1[IAP_USER_PARAM_RESERVED_SIZE]; /*!< 预留 */
} iap_user_boot_param_t;

_Static_assert(sizeof(iap_user_boot_param_t) <= 128,
               "iap_user_boot_param_t 必须 <= 128 字节 (RTC custom 区大小)，"
               "请与 IAP 端 iap_boot_param.h 同步");

/**
 * @brief 读取 RTC RAM 中的启动参数
 *
 * @param out 输出结构
 * @return ESP_OK 成功且有效; ESP_ERR_INVALID_STATE 无效 (冷启动/未写入)
 */
esp_err_t iap_user_boot_param_read(iap_user_boot_param_t *out);

/**
 * @brief 判断 RTC RAM 启动参数是否有效 (magic + CRC)
 */
bool iap_user_boot_param_valid(void);

/**
 * @brief 请求重启进入 IAP (v5)
 *
 * 写 RTC RAM boot_target=0 → esp_restart()。
 * 不再使用 esp_ota_set_boot_partition。
 *
 * @return ESP_OK (不会返回，函数内 esp_restart)
 */
esp_err_t iap_user_enter_iap(void);

/* ============================================================================
 * 配置更新请求 (v6)
 * ========================================================================== */
/*
 * 用户程序**不能直接写配置区**。需要持久化修改配置时:
 *
 *   // 1. 填写要修改的字段
 *   iap_user_request_boot_param("mode=debug;server=192.168.1.10", 0);
 *   iap_user_request_active_slot(1);
 *
 *   // 2. 提交 + 重启 (不会返回)
 *   iap_user_request_apply();
 *
 * IAP 启动时检测到请求 → 写入配置区 → 重启启动用户程序。
 *
 * 只能修改 IAP 暴露的白名单字段 (IAP_USER_UPD_ALLOWED_MASK)。
 */

/**
 * @brief 请求修改启动参数字符串
 *
 * @param param 参数字符串，NULL 表示清空
 * @param len   有效字符数；传 0 表示用 strlen
 * @return ESP_OK 成功
 */
esp_err_t iap_user_request_boot_param(const char *param, size_t len);

/**
 * @brief 请求修改 OTA 槽序号
 *
 * @param slot 目标槽序号 (0 = ota_0)
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 槽序号越界
 */
esp_err_t iap_user_request_active_slot(uint8_t slot);

/**
 * @brief 请求修改加载地址
 *
 * @param addr 目标地址 (0 = 恢复默认; 非 0 须 >= 0x140000)
 * @return ESP_OK 成功; ESP_ERR_INVALID_ARG 地址非法
 */
esp_err_t iap_user_request_user_addr(uint32_t addr);

/**
 * @brief 请求修改启动目标
 *
 * @param target IAP_USER_BOOT_TARGET_IAP / _APP
 * @return ESP_OK 成功
 */
esp_err_t iap_user_request_boot_target(uint8_t target);

/**
 * @brief 提交更新请求 (重算 CRC)
 *
 * 调用 iap_user_request_*() 后**必须**调用本函数。
 *
 * @return ESP_OK 成功
 */
esp_err_t iap_user_request_commit(void);

/**
 * @brief 提交请求并重启进 IAP 应用配置 (不返回)
 *
 * 等价于 iap_user_request_commit() + esp_restart()。
 */
void iap_user_request_apply(void);

/**
 * @brief 查询是否有待处理的更新请求
 */
bool iap_user_has_update_request(void);

/**
 * @brief 放弃更新请求 (清除 update_flags)
 *
 * @return ESP_OK 成功
 */
esp_err_t iap_user_request_clear(void);

/**
 * @brief 示例主函数，演示完整交互流程
 */
void iap_user_example_main(void);

#ifdef __cplusplus
}
#endif
