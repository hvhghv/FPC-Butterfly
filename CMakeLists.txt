# 蝴蝶板灯珠 HTTP 控制应用 (ESP32-C6)
#
# 编译: idf.py set-target esp32c6 && idf.py build
# 打包: python build_user_app.py --target esp32c6 --name led_butterfly --version 1.0.0

cmake_minimum_required(VERSION 3.16)

# 工程名 (会体现在 iap_pack.py 的文件头 project_name 中)
set(PROJECT_NAME "led_butterfly")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_NAME})
