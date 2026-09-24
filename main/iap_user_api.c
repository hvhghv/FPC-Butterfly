/*
 * ESP IAP - 用户程序侧集成示例 (v5)
 *
 * 本文件演示用户程序如何与 IAP 交互:
 *   1. 读取 RTC RAM 启动参数 (IAP 启动用户程序前写入)
 *   2. 请求重启进入 IAP 下载模式 (写 RTC RAM → 重启)
 *   3. 读取启动参数字符串 (key=value 对)
 *
 * ⚠️ v5: **用户程序不允许访问配置区 (0x8000)**。
 *
 * 原因:
 *   - 配置区位于 IDF flash 写保护区 (0x0 ~ 0xBFFF)，直写会 abort()
 *   - 配置区是 IAP 私有数据，用户程序读写会破坏双槽冗余一致性
 *
 * 因此本文件**不含**任何配置区读写代码。用户程序需要配置时:
 *   - 只读: iap_user_boot_param_read() / iap_user_get_boot_param()
 *   - 请求修改: iap_user_request_download() → 重启进 IAP 修改
 *
 * 用户程序只需把本文件加入自己的工程即可，无需与 IAP 共用分区表。
 *
 * 典型用法:
 *
 *   // 收到升级指令时，请求重启进入 IAP 下载模式
 *   iap_user_request_download();
 *
 *   // 启动时读取 IAP 传入的启动参数
 *   char param[256];
 *   if (iap_user_get_boot_param(param, sizeof(param)) == ESP_OK) {
 *       ESP_LOGI(TAG, "启动参数: %s", param);
 *   }
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"
#include "esp_image_format.h"   /* rtc_retain_mem_t (v5 RTC 参数) */
#include "bootloader_common.h"  /* bootloader_common_get_rtc_retain_mem() */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "iap_user_api.h"

static const char *TAG = "user_app";

/* ============================================================================
 * v5: 用户程序**不访问配置区**
 *
 * 配置区 (0x8000) 在 IDF flash 写保护区 (0x0 ~ 0xBFFF) 内，
 * 且属于 IAP 私有数据。用户程序通过 RTC RAM 启动参数只读获取信息，
 * 通过 iap_user_request_download() 请求 IAP 代为修改配置。
 * ========================================================================== */

/* -------------------------------------------------------------------------- */
/* v5: 配置区访问已移除                                                        */
/* -------------------------------------------------------------------------- */

/*
 * 用户程序**不访问配置区** (0x8000):
 *   - 配置区在 IDF flash 写保护区 (0x0 ~ 0xBFFF) 内
 *   - 配置区是 IAP 私有数据，双槽冗余一致性由 IAP 维护
 *
 * 早期版本的 iap_user_config_read() / iap_user_config_write() 已**彻底删除**
 * (连同 iap_user_cfg_t 结构)。用户程序需要配置请用 RTC RAM 启动参数 API，
 * 需要修改配置请用 iap_user_request_download() 重启进 IAP。
 */

/* ============================================================================
 * RTC RAM 启动参数 (v5)
 * ========================================================================== */

/**
 * @brief 获取 RTC RAM 中的启动参数指针
 */
static iap_user_boot_param_t *user_param_get(void)
{
    rtc_retain_mem_t *mem = bootloader_common_get_rtc_retain_mem();
    return (iap_user_boot_param_t *)mem->custom;
}

bool iap_user_boot_param_valid(void)
{
    iap_user_boot_param_t *p = user_param_get();
    if (p->magic != IAP_USER_PARAM_MAGIC) {
        return false;
    }
    uint32_t crc = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                    offsetof(iap_user_boot_param_t, crc32));
    return crc == p->crc32;
}

esp_err_t iap_user_boot_param_read(iap_user_boot_param_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    iap_user_boot_param_t *p = user_param_get();
    if (!iap_user_boot_param_valid()) {
        return ESP_ERR_INVALID_STATE;
    }
    *out = *p;
    return ESP_OK;
}

/**
 * @brief 请求重启进入 IAP (v5)
 *
 * 写 RTC RAM boot_target=0 → esp_restart()。
 * 自定义 bootloader 读 RTC RAM 后会启动 IAP。
 *
 * 注意: 冷启动时 RTC RAM 本就无效, bootloader 默认启动 IAP,
 *       因此本函数只是"明确表达"回 IAP 的意图。
 */
esp_err_t iap_user_enter_iap(void)
{
    iap_user_boot_param_t *p = user_param_get();

    p->magic       = IAP_USER_PARAM_MAGIC;
    p->version     = 1;
    p->boot_target = IAP_USER_BOOT_TARGET_IAP;
    p->crc32 = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                offsetof(iap_user_boot_param_t, crc32));

    ESP_LOGW(TAG, "已请求进入 IAP，即将重启...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;  /* 不会执行到这里 */
}

/**
 * @brief 请求重启进入 IAP 下载模式
 *
 * v5: 只写 RTC RAM (boot_target=IAP) → 重启。
 *     不再写配置区 (配置区对用户程序不开放)。
 *
 * @return ESP_OK 表示已设置标志并即将重启 (不会返回)
 */
esp_err_t iap_user_request_download(void)
{
    iap_user_boot_param_t *p = user_param_get();
    p->magic       = IAP_USER_PARAM_MAGIC;
    p->version     = 1;
    p->boot_target = IAP_USER_BOOT_TARGET_IAP;
    p->crc32 = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                offsetof(iap_user_boot_param_t, crc32));

    ESP_LOGW(TAG, "已请求进入 IAP 下载模式，即将重启...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;  /* 不会执行到这里 */
}

/**
 * @brief 检查是否处于 IAP 下载模式
 *
 * v5: 通过 RTC RAM 启动参数判定 (boot_target == IAP)。
 *
 * @return true 表示 IAP 处于下载模式
 */
bool iap_user_is_download_requested(void)
{
    iap_user_boot_param_t *p = user_param_get();
    if (!iap_user_boot_param_valid()) {
        return false;
    }
    return p->boot_target == IAP_USER_BOOT_TARGET_IAP;
}

/* -------------------------------------------------------------------------- */
/* 配置更新请求 (v6)                                                           */
/* -------------------------------------------------------------------------- */
/*
 * 用户程序不能直接写配置区。把要修改的值写入 RTC RAM 的 update 区，
 * 重启进 IAP 后由 IAP 写入配置区持久化，再重启启动用户程序。
 */

/* 加载地址下限 (与 IAP 端 IAP_FIXED_REGION_END 一致) */
#define IAP_USER_FIXED_REGION_END   0x140000U

/** 保证 RTC RAM 参数区已初始化 (magic/version 有效) */
static void user_param_ensure_init(void)
{
    iap_user_boot_param_t *p = user_param_get();
    if (p->magic != IAP_USER_PARAM_MAGIC) {
        /*
         * 冷启动: RTC RAM 内容随机。只写 magic/version，
         * 其余字段保持原样 (bootloader 会在下次启动时重写)。
         */
        p->magic   = IAP_USER_PARAM_MAGIC;
        p->version = 1;
    }
}

esp_err_t iap_user_request_boot_param(const char *param, size_t len)
{
    iap_user_boot_param_t *p = user_param_get();
    user_param_ensure_init();

    if (param == NULL) {
        p->upd_param_len = 0;
        p->upd_param[0]  = '\0';
    } else {
        if (len == 0) {
            len = strlen(param);
        }
        if (len >= IAP_USER_PARAM_STR_SIZE) {
            ESP_LOGW(TAG, "启动参数过长 (%u >= %u)，将截断",
                     (unsigned)len, (unsigned)IAP_USER_PARAM_STR_SIZE);
            len = IAP_USER_PARAM_STR_SIZE - 1;
        }
        memcpy(p->upd_param, param, len);
        p->upd_param[len] = '\0';
        p->upd_param_len  = (uint8_t)len;
    }

    p->update_flags |= IAP_USER_UPD_BOOT_PARAM;
    return ESP_OK;
}

esp_err_t iap_user_request_active_slot(uint8_t slot)
{
    iap_user_boot_param_t *p = user_param_get();
    user_param_ensure_init();

    /* 槽上限由 IAP 侧最终校验 (IAP 才知道实际槽数) */
    if (slot >= 16) {
        ESP_LOGE(TAG, "OTA 槽序号越界: %u (最大 15)", (unsigned)slot);
        return ESP_ERR_INVALID_ARG;
    }

    p->upd_active_slot = slot;
    p->update_flags   |= IAP_USER_UPD_ACTIVE_SLOT;
    return ESP_OK;
}

esp_err_t iap_user_request_user_addr(uint32_t addr)
{
    iap_user_boot_param_t *p = user_param_get();
    user_param_ensure_init();

    /* 0 = 恢复默认; 非 0 必须落在可变区 (不覆盖固定区) */
    if (addr != 0 && addr < IAP_USER_FIXED_REGION_END) {
        ESP_LOGE(TAG, "加载地址非法: 0x%08" PRIX32 " (须为 0 或 >= 0x%06X)",
                 addr, (unsigned)IAP_USER_FIXED_REGION_END);
        return ESP_ERR_INVALID_ARG;
    }

    p->upd_user_addr = addr;
    p->update_flags |= IAP_USER_UPD_USER_ADDR;
    return ESP_OK;
}

esp_err_t iap_user_request_boot_target(uint8_t target)
{
    iap_user_boot_param_t *p = user_param_get();
    user_param_ensure_init();

    if (target != IAP_USER_BOOT_TARGET_IAP && target != IAP_USER_BOOT_TARGET_APP) {
        ESP_LOGE(TAG, "启动目标非法: %u", (unsigned)target);
        return ESP_ERR_INVALID_ARG;
    }

    p->upd_boot_target = target;
    p->update_flags   |= IAP_USER_UPD_BOOT_TARGET;
    return ESP_OK;
}

esp_err_t iap_user_request_commit(void)
{
    iap_user_boot_param_t *p = user_param_get();
    user_param_ensure_init();

    /* 只保留白名单内的位 */
    p->update_flags &= IAP_USER_UPD_ALLOWED_MASK;

    if (p->update_flags == 0) {
        ESP_LOGW(TAG, "无有效更新字段 (update_flags=0)");
        return ESP_ERR_INVALID_STATE;
    }

    p->crc32 = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                offsetof(iap_user_boot_param_t, crc32));
    ESP_LOGI(TAG, "更新请求已提交 (flags=0x%02X)", (unsigned)p->update_flags);
    return ESP_OK;
}

void iap_user_request_apply(void)
{
    if (iap_user_request_commit() != ESP_OK) {
        ESP_LOGW(TAG, "无有效更新字段，仍将重启进 IAP");
    }

    /* 标记启动目标为 IAP，bootloader 会启动 IAP */
    iap_user_boot_param_t *p = user_param_get();
    p->boot_target = IAP_USER_BOOT_TARGET_IAP;
    p->crc32 = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                offsetof(iap_user_boot_param_t, crc32));

    ESP_LOGW(TAG, "即将重启进 IAP 应用配置更新...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

bool iap_user_has_update_request(void)
{
    iap_user_boot_param_t *p = user_param_get();
    if (!iap_user_boot_param_valid()) {
        return false;
    }
    return (p->update_flags & IAP_USER_UPD_ALLOWED_MASK) != 0;
}

esp_err_t iap_user_request_clear(void)
{
    iap_user_boot_param_t *p = user_param_get();
    p->update_flags = 0;
    p->crc32 = esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)p,
                                offsetof(iap_user_boot_param_t, crc32));
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* 启动参数字符串                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief 读取启动参数字符串
 *
 * v5: 启动参数来自 **RTC RAM** (IAP 启动用户程序前写入),
 *     不再从配置区读取 (配置区对用户程序不开放)。
 *
 * 参数格式为 "key1=value1;key2=value2"。
 */
esp_err_t iap_user_get_boot_param(char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    buf[0] = '\0';

    iap_user_boot_param_t bp;
    if (iap_user_boot_param_read(&bp) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    /* 未设置 */
    if (bp.param_len == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    /* 长度字段越界保护 */
    /* 长度字段越界保护 (param 缓冲区为 IAP_USER_PARAM_STR_SIZE) */
    if (bp.param_len >= IAP_USER_PARAM_STR_SIZE) {
        ESP_LOGW(TAG, "启动参数长度字段非法: %u", (unsigned)bp.param_len);
        return ESP_ERR_INVALID_SIZE;
    }

    if (buf_len <= (size_t)bp.param_len) {
        ESP_LOGE(TAG, "缓冲区不足: %u <= %u", (unsigned)buf_len, (unsigned)bp.param_len);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(buf, bp.param, bp.param_len);
    buf[bp.param_len] = '\0';
    return ESP_OK;
}

/**
 * @brief 读取启动参数并标记为已消费
 *
 * v5: RTC RAM 中的参数由 IAP 每次启动时重写，无需用户程序回写。
 *     本函数与 iap_user_get_boot_param() 等价，仅为语义清晰保留。
 */
esp_err_t iap_user_take_boot_param(char *buf, size_t buf_len)
{
    return iap_user_get_boot_param(buf, buf_len);
}

/**
 * @brief 从启动参数中提取指定键的值
 *
 * 参数格式为 "key1=value1;key2=value2"。
 */
esp_err_t iap_user_boot_param_get_value(const char *param, const char *key,
                                        char *out, size_t out_len)
{
    if (param == NULL || key == NULL || out == NULL || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    out[0] = '\0';

    size_t key_len = strlen(key);
    if (key_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *p = param;

    while (*p != '\0') {
        /* 跳过前导分隔符与空白 */
        while (*p == ';' || *p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == '\0') {
            break;
        }

        /* 当前键值对的结束位置 */
        const char *end = strchr(p, ';');
        if (end == NULL) {
            end = p + strlen(p);
        }

        /* 查找 '=' */
        const char *eq = memchr(p, '=', (size_t)(end - p));
        if (eq != NULL) {
            size_t cur_key_len = (size_t)(eq - p);

            /* 去除键尾部空白 */
            while (cur_key_len > 0 &&
                   (p[cur_key_len - 1] == ' ' || p[cur_key_len - 1] == '\t')) {
                cur_key_len--;
            }

            if (cur_key_len == key_len && strncmp(p, key, key_len) == 0) {
                /* 命中: 复制值 */
                const char *val = eq + 1;
                size_t val_len = (size_t)(end - val);

                /* 去除值首尾空白 */
                while (val_len > 0 && (*val == ' ' || *val == '\t')) {
                    val++;
                    val_len--;
                }
                while (val_len > 0 &&
                       (val[val_len - 1] == ' ' || val[val_len - 1] == '\t')) {
                    val_len--;
                }

                if (val_len >= out_len) {
                    return ESP_ERR_INVALID_SIZE;
                }
                memcpy(out, val, val_len);
                out[val_len] = '\0';
                return ESP_OK;
            }
        }

        p = end;
        if (*p == ';') {
            p++;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief 查询启动参数是否有效
 *
 * v5: 从 RTC RAM 判定。
 */
bool iap_user_has_boot_param(void)
{
    iap_user_boot_param_t bp;
    if (iap_user_boot_param_read(&bp) != ESP_OK) {
        return false;
    }
    return bp.param_len > 0 && bp.param_len < IAP_USER_PARAM_STR_SIZE;
}

/**
 * @brief 查询启动参数是否已被消费
 *
 * v5: RTC RAM 参数由 IAP 每次启动重写，用户程序无需标记消费。
 *     本函数仅为保持符号兼容，始终返回 false。
 */
bool iap_user_boot_param_consumed(void)
{
    return false;
}

/**
 * @brief 示例: 用户程序主函数
 *
 * 演示与 IAP 的完整交互流程 (v5)。
 */
void iap_user_example_main(void)
{
    ESP_LOGI(TAG, "用户程序已启动");

    /* 1. 读取启动参数并据此初始化 (来自 RTC RAM) */
    char param[IAP_USER_BOOT_PARAM_SIZE];
    esp_err_t perr = iap_user_get_boot_param(param, sizeof(param));
    if (perr == ESP_OK) {
        ESP_LOGI(TAG, "启动参数: %s", param);

        /* 提取各键的值 */
        char val[32];

        if (iap_user_boot_param_get_value(param, "mode", val, sizeof(val)) == ESP_OK) {
            ESP_LOGI(TAG, "  运行模式: %s", val);
            /* 根据模式初始化:
             *   if (strcmp(val, "debug") == 0) { ... }
             */
        }

        if (iap_user_boot_param_get_value(param, "reason", val, sizeof(val)) == ESP_OK) {
            ESP_LOGI(TAG, "  启动原因: %s", val);
        }

        if (iap_user_boot_param_get_value(param, "boot", val, sizeof(val)) == ESP_OK) {
            ESP_LOGI(TAG, "  启动计数: %s", val);
        }

        if (iap_user_boot_param_get_value(param, "baud", val, sizeof(val)) == ESP_OK) {
            ESP_LOGI(TAG, "  通信波特率: %s", val);
        }
    } else if (perr == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "无启动参数，使用默认配置");
    } else {
        ESP_LOGW(TAG, "读取启动参数失败: %s", esp_err_to_name(perr));
    }

    /* 2. 读取 RTC RAM 启动参数 (v5) */
    iap_user_boot_param_t bp;
    if (iap_user_boot_param_read(&bp) == ESP_OK) {
        ESP_LOGI(TAG, "RTC 启动参数: target=%u, addr=0x%08" PRIX32
                 ", size=%" PRIu32 ", ver=0x%06" PRIX32,
                 bp.boot_target, bp.user_addr, bp.user_size, bp.user_version);
    } else {
        ESP_LOGI(TAG, "无 RTC 启动参数 (冷启动)");
    }

    /* 3. 主循环 */
    while (1) {
        ESP_LOGI(TAG, "用户程序运行中... 空闲堆 %" PRIu32,
                 (uint32_t)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000));

        /*
         * 示例 A: 请求升级 —— 重启进 IAP 下载模式
         *   iap_user_request_download();
         *
         * 示例 B: 请求修改配置 (v6) —— 重启进 IAP 写入配置区
         *   iap_user_request_boot_param("mode=debug;server=192.168.1.10", 0);
         *   iap_user_request_active_slot(1);
         *   iap_user_request_apply();   // 提交 + 重启 (不返回)
         */
    }
}
