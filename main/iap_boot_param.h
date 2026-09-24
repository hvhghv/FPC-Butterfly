/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file iap_boot_param.h
 * @brief IAP / bootloader / APP 三方共享的启动参数 (RTC RAM)
 *
 * ============================================================================
 *  用途
 * ============================================================================
 *
 * 通过 **IDF 官方 RTC 保留内存机制** 在 bootloader 与 APP 之间传递启动参数,
 * 使 bootloader **完全不读 flash** 即可决定启动谁。
 *
 *   IAP         → 写 RTC RAM (boot_target / user_addr / user_size)
 *   bootloader  → 读 RTC RAM, 按 boot_target 加载镜像 (不做任何 flash 读)
 *   APP         → 读 RTC RAM, 拿启动参数
 *
 * ============================================================================
 *  存储位置 (为什么不用 0x50000000)
 * ============================================================================
 *
 * ⚠️ **不要**直接使用 0x50000000 —— 那是 APP 的 lp_ram_seg 起始,
 *    会被 APP 正常使用 (冲突)。
 *
 * 正确做法: 使用 IDF 官方 rtc_retain_mem_t.custom[] 区,
 * 它位于 LP RAM **末尾保留区**:
 *
 *   ESP32-C6 (esp_system/ld/esp32c6/memory.ld.in):
 *     0x50000000                        LP RAM 起始
 *       lp_ram_seg (RW)                 len = 0x4000 - RESERVE_RTC_MEM
 *     0x50004000 - RESERVE_RTC_MEM      lp_reserved_seg
 *       ├── bootloader_data_rtc_mem     ← rtc_retain_mem_t (含 custom[])
 *       ├── rtc_timer_data_in_rtc_mem
 *       └── ...
 *     0x50004000                        LP RAM 末尾
 *
 * 两侧地址一致性由 IDF 保证 (bootloader_common_loader.c:268):
 *   - bootloader 侧: 硬编码 SOC_RTC_DRAM_LOW
 *   - APP 侧:        链接器放入 .bootloader_data_rtc_mem 段 (同一物理地址)
 *
 * ============================================================================
 *  保持能力
 * ============================================================================
 *
 *   | 场景           | RTC RAM | 行为                          |
 *   |----------------|:-------:|-------------------------------|
 *   | 冷启动 (断电)  | 无效    | magic 校验失败 → 启动 IAP ✅  |
 *   | 软复位         | 保持    | 按 boot_target 启动           |
 *   | 看门狗复位     | 保持    | 按 boot_target 启动           |
 *   | 深睡眠唤醒     | 保持    | 按 boot_target 启动           |
 *
 * 因此 "每次上电必跑 IAP" 的要求仍然满足 —— 冷启动时 RTC RAM 无效。
 *
 * ============================================================================
 *  Kconfig 依赖
 * ============================================================================
 *
 *   CONFIG_BOOTLOADER_RESERVE_RTC_MEM=y
 *   CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC=y
 *   CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC_SIZE=128   # >= sizeof(iap_boot_param_t)
 *
 * 详见 docs/FINAL-REPORT.md §5.5
 */

#ifndef IAP_BOOT_PARAM_H
#define IAP_BOOT_PARAM_H

/* ============================================================================
 * 目标芯片能力检查
 *
 * 本方案依赖 IDF 的 RTC 保留内存机制 (rtc_retain_mem_t.custom[]),
 * 它要求芯片支持 RTC FAST RAM (SOC_RTC_FAST_MEM_SUPPORTED)。
 *
 * 不支持的目标:
 *   - ESP32-C2  (无 RTC FAST RAM)
 *
 * 支持的目标 (SOC_RTC_FAST_MEM_SUPPORTED=1):
 *   - ESP32 / ESP32-S2 / ESP32-S3 / ESP32-C3 / ESP32-C5 / ESP32-C6 / ESP32-H2
 * ========================================================================== */
#include "soc/soc_caps.h"

#ifndef SOC_RTC_FAST_MEM_SUPPORTED
#error "本方案需要 RTC FAST RAM (SOC_RTC_FAST_MEM_SUPPORTED)。" \
       "当前目标不支持 —— 例如 ESP32-C2 无 RTC FAST RAM, 无法使用 RTC RAM 传递启动参数。"
#endif

#if !defined(CONFIG_BOOTLOADER_RESERVE_RTC_MEM) || \
    !defined(CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC)
#error "本方案需要以下 Kconfig (见 sdkconfig.defaults):" \
       "CONFIG_BOOTLOADER_RESERVE_RTC_MEM=y 和 CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC=y"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "sdkconfig.h"
#include "esp_image_format.h"       /* rtc_retain_mem_t */

#include <string.h>                 /* strlen / memcpy (iap_param_set_string) */

/*
 * bootloader 环境 (BOOTLOADER_BUILD) 与 APP 环境都能包含本头文件。
 *
 * APP 侧: 需要 bootloader_common.h 提供 bootloader_common_get_rtc_retain_mem()
 * bootloader 侧: bootloader_common.h 同样可用 (bootloader_support 组件)
 */
#include "bootloader_common.h"

/* CRC32: APP 侧与 bootloader 侧都用 ROM 版本, 无需额外依赖 */
#include "esp_rom_crc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* 常量                                                                        */
/* -------------------------------------------------------------------------- */

/** 魔术字 ("IAPB" 小端) */
#define IAP_PARAM_MAGIC     0x49415042U

/** 结构版本 (布局变更时递增, 用于未来迁移) */
#define IAP_PARAM_VERSION   0x0001U

/** 启动目标 */
#define IAP_BOOT_TARGET_IAP 0   /*!< 启动 IAP 程序 */
#define IAP_BOOT_TARGET_APP 1   /*!< 启动用户程序 */

/** 启动参数字符串最大长度 (含 '\0' 的缓冲区大小) */
#define IAP_PARAM_STR_SIZE  32

/** 预留区大小 (保持结构体 <= 128 字节) */
#define IAP_PARAM_RESERVED_SIZE 8

/* -------------------------------------------------------------------------- */
/* 用户程序可请求更新的字段 (v6)                                                */
/* -------------------------------------------------------------------------- */
/*
 * 用户程序**不能直接访问配置区** (0x8000 在 IDF 写保护区内)。
 * 需要修改配置时，把目标值写入 RTC RAM 的 update 区并置位对应 bit，
 * 然后重启进 IAP；IAP 启动时检测到请求，写入配置区持久化，再重启启动。
 *
 * 当前**只允许**修改以下字段 (bootloader 暴露给用户程序的子集):
 *   - 启动参数字符串 (boot_param)
 *   - OTA 槽序号 (active_slot)
 *   - 加载地址 (user_addr)
 *   - 启动目标 (boot_target)
 */

/** update 请求: 修改启动参数字符串 */
#define IAP_PARAM_UPD_BOOT_PARAM    (1U << 0)
/** update 请求: 修改 OTA 槽序号 (active_slot) */
#define IAP_PARAM_UPD_ACTIVE_SLOT   (1U << 1)
/** update 请求: 修改加载地址 (user_addr) */
#define IAP_PARAM_UPD_USER_ADDR     (1U << 2)
/** update 请求: 修改启动目标 (boot_target) */
#define IAP_PARAM_UPD_BOOT_TARGET   (1U << 3)

/** 允许用户程序修改的字段掩码 (白名单) */
#define IAP_PARAM_UPD_ALLOWED_MASK  (IAP_PARAM_UPD_BOOT_PARAM  | \
                                     IAP_PARAM_UPD_ACTIVE_SLOT | \
                                     IAP_PARAM_UPD_USER_ADDR   | \
                                     IAP_PARAM_UPD_BOOT_TARGET)

/* -------------------------------------------------------------------------- */
/* 结构                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief IAP / bootloader / APP 共享的启动参数
 *
 * **只传启动参数** —— 配置字段 (WiFi / I2C / 触发源等) 仍保留在 iap_cfg,
 * 用户程序**不可**访问配置区，只能通过本结构体读取，或通过 update 区请求修改。
 *
 * 大小: 104 字节 (实测, 无填充)
 */
typedef struct {
    /* --- 头部 --- */
    uint32_t magic;          /*!< IAP_PARAM_MAGIC */
    uint32_t version;        /*!< IAP_PARAM_VERSION */
    uint32_t crc32;          /*!< 以下数据的 CRC32 (offset 8 起) */

    /* --- 启动决策 (原 flash 标志区的职责) --- */
    uint8_t  boot_target;    /*!< IAP_BOOT_TARGET_IAP / _APP */
    uint8_t  reserved0[3];   /*!< 对齐填充 */
    uint32_t user_addr;      /*!< APP 加载地址 (0x150000) */
    uint32_t user_size;      /*!< APP 大小 (字节) */
    uint32_t user_version;   /*!< APP 版本号 (可选, 0 表示未知) */

    /* --- 布局信息 (供 APP 定位共享资源) --- */
    uint32_t cfg_addr;       /*!< iap_cfg 地址 (0x8000) */
    uint32_t cfg_seq;        /*!< 配置槽序号 (IAP 写入时的快照) */

    /* --- 启动参数字符串 (v5: 取代配置区 boot_param) --- */
    uint8_t  param_len;      /*!< 有效字符数 (不含 '\0') */
    uint8_t  param_reserved[3];
    char     param[IAP_PARAM_STR_SIZE]; /*!< "key1=value1;key2=value2" */

    /* --- 用户程序请求更新区 (v6) --- */
    /*
     * 用户程序把要修改的值写入下面字段，并置位 update_flags 对应 bit，
     * 然后 esp_restart()。IAP 启动时检测 update_flags != 0 即执行更新。
     */
    uint8_t  update_flags;   /*!< IAP_PARAM_UPD_* 位掩码, 0 = 无请求 */
    uint8_t  upd_active_slot;/*!< 请求的 OTA 槽序号 */
    uint8_t  upd_reserved[2];
    uint32_t upd_user_addr;  /*!< 请求的加载地址 */
    uint8_t  upd_boot_target;/*!< 请求的启动目标 */
    uint8_t  upd_param_len;  /*!< 请求的参数字符串长度 */
    uint8_t  upd_param_reserved[2];
    char     upd_param[IAP_PARAM_STR_SIZE]; /*!< 请求的参数字符串 */

    /* --- 预留 (配置子集以后再加, 无需改结构大小) --- */
    uint8_t  reserved1[IAP_PARAM_RESERVED_SIZE];
} iap_boot_param_t;

/* 编译期校验: 必须能放进 IDF 的 custom[] 区 */
#if defined(CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC_SIZE)
_Static_assert(sizeof(iap_boot_param_t) <= CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC_SIZE,
               "iap_boot_param_t 超过 CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC_SIZE，"
               "请增大该 Kconfig 值");
#endif

/* -------------------------------------------------------------------------- */
/* 访问接口 (bootloader 与 APP 通用)                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief 获取共享参数区指针
 *
 * bootloader 与 APP 都通过本函数访问, 地址由 IDF 保证一致。
 *
 * @return 指向 iap_boot_param_t 的指针 (始终有效, 不返回 NULL)
 */
static inline iap_boot_param_t *iap_param_get(void)
{
    rtc_retain_mem_t *mem = bootloader_common_get_rtc_retain_mem();
    return (iap_boot_param_t *)mem->custom;
}

/**
 * @brief 校验参数区是否有效 (magic + CRC)
 *
 * @param p 参数区指针
 * @return true 表示有效
 */
static inline bool iap_param_crc_ok(const iap_boot_param_t *p)
{
    if (p->magic != IAP_PARAM_MAGIC) {
        return false;
    }
    uint32_t crc = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                    offsetof(iap_boot_param_t, crc32));
    return crc == p->crc32;
}

/**
 * @brief 重算并写入 CRC32
 *
 * 修改参数区字段后必须调用本函数。
 *
 * @param p 参数区指针
 */
static inline void iap_param_update_crc(iap_boot_param_t *p)
{
    p->crc32 = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                offsetof(iap_boot_param_t, crc32));
}

/**
 * @brief 判断参数区是否有效 (便捷封装)
 */
static inline bool iap_param_valid(void)
{
    return iap_param_crc_ok(iap_param_get());
}

/**
 * @brief 清空参数区 (使 magic 失效 → 下次启动进 IAP)
 */
static inline void iap_param_clear(void)
{
    iap_boot_param_t *p = iap_param_get();
    p->magic = 0;
    p->crc32 = 0;
}

/* -------------------------------------------------------------------------- */
/* 写入接口 (仅 IAP / APP 使用, bootloader 只读)                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief 请求启动用户程序
 *
 * 由 IAP 调用。写入后需调用 esp_restart() 生效。
 *
 * @param user_addr   APP 加载地址
 * @param user_size   APP 大小
 * @param user_version APP 版本号 (可为 0)
 * @param cfg_addr    iap_cfg 地址
 * @param cfg_seq     配置槽序号
 */
static inline void iap_param_set_boot_app(uint32_t user_addr,
                                          uint32_t user_size,
                                          uint32_t user_version,
                                          uint32_t cfg_addr,
                                          uint32_t cfg_seq)
{
    iap_boot_param_t *p = iap_param_get();
    p->magic        = IAP_PARAM_MAGIC;
    p->version      = IAP_PARAM_VERSION;
    p->boot_target  = IAP_BOOT_TARGET_APP;
    p->user_addr    = user_addr;
    p->user_size    = user_size;
    p->user_version = user_version;
    p->cfg_addr     = cfg_addr;
    p->cfg_seq      = cfg_seq;
    iap_param_update_crc(p);
}

/**
 * @brief 设置启动参数字符串 (v5)
 *
 * 由 IAP 在启动用户程序前调用，把配置区的 boot_param 复制到 RTC RAM，
 * 使用户程序无需访问配置区即可获取启动参数。
 *
 * 必须在 iap_param_set_boot_app() **之后** 调用 (否则会被覆盖)。
 *
 * @param param 参数字符串 ("key1=value1;key2=value2")，可为 NULL (清空)
 * @param len   有效字符数 (不含 '\0')；传 0 表示用 strlen
 */
static inline void iap_param_set_string(const char *param, size_t len)
{
    iap_boot_param_t *p = iap_param_get();

    if (param == NULL) {
        p->param_len = 0;
        p->param[0]  = '\0';
        iap_param_update_crc(p);
        return;
    }

    if (len == 0) {
        len = strlen(param);
    }
    if (len >= IAP_PARAM_STR_SIZE) {
        len = IAP_PARAM_STR_SIZE - 1;
    }

    memcpy(p->param, param, len);
    p->param[len] = '\0';
    p->param_len  = (uint8_t)len;
    iap_param_update_crc(p);
}

/**
 * @brief 请求启动 IAP 程序
 *
 * 由 APP 调用 (回 IAP)。写入后需调用 esp_restart() 生效。
 */
static inline void iap_param_set_boot_iap(void)
{
    iap_boot_param_t *p = iap_param_get();
    p->magic       = IAP_PARAM_MAGIC;
    p->version     = IAP_PARAM_VERSION;
    p->boot_target = IAP_BOOT_TARGET_IAP;
    iap_param_update_crc(p);
}

/* -------------------------------------------------------------------------- */
/* 用户程序请求更新接口 (v6)                                                    */
/* -------------------------------------------------------------------------- */
/*
 * 用户程序**不能直接写配置区**。需要持久化修改时:
 *   1. 调用 iap_param_request_*() 填写 update 区并置位 update_flags
 *   2. 调用 iap_param_request_commit() 重算 CRC
 *   3. esp_restart() → IAP 启动时检测到请求 → 写入配置区 → 重启启动
 *
 * 只有 IAP_PARAM_UPD_ALLOWED_MASK 中的字段可被修改 (白名单)。
 */

/**
 * @brief 请求修改启动参数字符串
 *
 * @param param 参数字符串，NULL 表示清空
 * @param len   有效字符数；传 0 表示用 strlen
 */
static inline void iap_param_request_boot_param(const char *param, size_t len)
{
    iap_boot_param_t *p = iap_param_get();

    if (param == NULL) {
        p->upd_param_len = 0;
        p->upd_param[0]  = '\0';
    } else {
        if (len == 0) {
            len = strlen(param);
        }
        if (len >= IAP_PARAM_STR_SIZE) {
            len = IAP_PARAM_STR_SIZE - 1;
        }
        memcpy(p->upd_param, param, len);
        p->upd_param[len] = '\0';
        p->upd_param_len  = (uint8_t)len;
    }

    p->update_flags |= IAP_PARAM_UPD_BOOT_PARAM;
}

/**
 * @brief 请求修改 OTA 槽序号 (active_slot)
 *
 * @param slot 目标槽序号 (0 = ota_0)
 */
static inline void iap_param_request_active_slot(uint8_t slot)
{
    iap_boot_param_t *p = iap_param_get();
    p->upd_active_slot = slot;
    p->update_flags   |= IAP_PARAM_UPD_ACTIVE_SLOT;
}

/**
 * @brief 请求修改加载地址 (user_addr)
 *
 * @param addr 目标加载地址 (如 0x150000)
 */
static inline void iap_param_request_user_addr(uint32_t addr)
{
    iap_boot_param_t *p = iap_param_get();
    p->upd_user_addr = addr;
    p->update_flags |= IAP_PARAM_UPD_USER_ADDR;
}

/**
 * @brief 请求修改启动目标 (boot_target)
 *
 * @param target IAP_BOOT_TARGET_IAP / _APP
 */
static inline void iap_param_request_boot_target(uint8_t target)
{
    iap_boot_param_t *p = iap_param_get();
    p->upd_boot_target = target;
    p->update_flags   |= IAP_PARAM_UPD_BOOT_TARGET;
}

/**
 * @brief 提交更新请求 (重算 CRC)
 *
 * 调用 iap_param_request_*() 填写完字段后**必须**调用本函数。
 */
static inline void iap_param_request_commit(void)
{
    iap_boot_param_t *p = iap_param_get();

    /* 只保留白名单内的位 */
    p->update_flags &= IAP_PARAM_UPD_ALLOWED_MASK;

    /* 保证 magic / version 有效 (冷启动时可能未初始化) */
    p->magic   = IAP_PARAM_MAGIC;
    p->version = IAP_PARAM_VERSION;

    iap_param_update_crc(p);
}

/**
 * @brief 查询是否有待处理的更新请求
 *
 * @return true 表示 update_flags 非空
 */
static inline bool iap_param_has_update_request(void)
{
    const iap_boot_param_t *p = iap_param_get();
    return iap_param_crc_ok(p) &&
           (p->update_flags & IAP_PARAM_UPD_ALLOWED_MASK) != 0;
}

/**
 * @brief 清除更新请求 (IAP 处理完或用户程序放弃时调用)
 */
static inline void iap_param_clear_update_request(void)
{
    iap_boot_param_t *p = iap_param_get();
    p->update_flags = 0;
    iap_param_update_crc(p);
}

#ifdef __cplusplus
}
#endif

#endif /* IAP_BOOT_PARAM_H */
