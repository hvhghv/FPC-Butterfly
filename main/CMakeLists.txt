# 蝴蝶板灯珠 HTTP 控制应用 — main 组件
#
# 模块划分:
#   main.c            应用入口、IAP 对接、上电自检动画
#   iap_user_api.c    IAP 对接接口（从 IAP 工程复制）
#   iap_boot_param.h  RTC RAM 启动参数结构（IAP/bootloader/APP 三方共享）
#   led_ctrl.c        12 路双硬件 PWM 驱动 (LEDC + MCPWM) + 动态效果引擎
#   app_wifi.c        WiFi AP/STA/APSTA
#   app_cmd.c         命令层 (传输无关，HTTP 与 BLE 共用)
#   app_http.c        HTTP 服务 (传输层)
#   app_ble.c         BLE GATT 服务 (传输层)
#   battery_ip5108.c  IP5108 电量计 (I2C, 复用 IO12/IO13)
#
# Web 页面不再内嵌在 C 源码里，而是作为独立文件放在 web/ 下，
# 编译时由 IDF 的 EMBED_FILES 自动转成二进制并链接进来:
#   web/index.html -> _binary_index_html_start / _binary_index_html_end
# 这样改页面只需编辑 HTML，无需重新转义 C 字符串。
#
# 说明: esp_wifi 仅在支持 WiFi 的芯片上引入 (ESP32-H2 无 WiFi)。
#       这里用 IDF_TARGET 判断，因为 CONFIG_SOC_WIFI_SUPPORTED 是
#       soc_caps.h 中的宏，不是 Kconfig 选项，CMake 阶段不可用。

set(app_requires
    esp_partition
    esp_system
    esp_timer
    esp_rom
    nvs_flash
    esp_driver_ledc
    esp_driver_mcpwm
    esp_driver_gpio
    # IP5108 电量计通过 I2C 读取电池信息 (IO12=SCL / IO13=SDA)
    esp_driver_i2c
    # 需要从 USB Serial/JTAG 外设释放 IO12/IO13 (LL 层函数)
    esp_driver_usb_serial_jtag
    esp_netif
    esp_event
    esp_http_server
    lwip
    esp_app_format
    spi_flash
    # v5: iap_boot_param.h 需要 esp_image_format.h (rtc_retain_mem_t)
    #     与 bootloader_common.h (bootloader_common_get_rtc_retain_mem)
    bootloader_support
)

if(NOT IDF_TARGET STREQUAL "esp32h2")
    list(APPEND app_requires esp_wifi)
endif()

# BLE 仅在支持蓝牙的芯片上引入。
# 注意: esp32h2 有 BLE 但无 WiFi，因此判断条件是分开的。
# 这里用 IDF_TARGET 判断而非 SOC_BLE_SUPPORTED，因为后者是
# soc_caps.h 中的宏，CMake 阶段不可用。
if(NOT IDF_TARGET STREQUAL "esp32s2")
    list(APPEND app_requires bt)
endif()

idf_component_register(
    SRCS
        "main.c"
        "iap_user_api.c"
        "led_ctrl.c"
        "app_wifi.c"
        "app_cmd.c"
        "app_http.c"
        "app_ble.c"
        "battery_ip5108.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        ${app_requires}
    # 编译时把 web/ 下的静态资源嵌入为二进制符号 (无需外部工具)
    EMBED_FILES
        "web/index.html"
)

# ---------------------------------------------------------------------------
# 目录形式嵌入 (EMBED_TXTFILES / EMBED_FILES 的替代方案)
#
# 如果以后 web/ 下文件变多，可改用 target_add_binary_data() 一次性嵌入整个
# 目录，生成符号形如 _binary_<相对路径>_start / _end，例如:
#
#   target_add_binary_data(${COMPONENT_LIB} "web/index.html" TEXT)
#
# 注意: 该函数必须在 idf_component_register() 之后调用，且需要 IDF >= v5.0。
# ---------------------------------------------------------------------------
