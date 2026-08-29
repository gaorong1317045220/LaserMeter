cmake_minimum_required(VERSION 3.16)

set(COMPONENTS main lvgl__lvgl espressif__esp32-camera espressif__esp_jpeg espressif__esp_new_jpeg)
set(EXTRA_COMPONENT_DIRS
    # Pin the verified LVGL 8.4.0 component shipped with this source package.
    "${CMAKE_CURRENT_LIST_DIR}/managed_components/lvgl__lvgl"
    "${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__esp32-camera"
    "${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__esp_jpeg"
    "${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__esp_new_jpeg"
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(board_self_test)
