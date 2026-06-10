cmake_minimum_required(VERSION 3.16)

# Read version from version.txt (written by version.py pre-script) so
# esp_app_desc_t.version matches the FIRMWARE_VERSION define.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(pool-switch-esp32c3)
