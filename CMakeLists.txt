cmake_minimum_required(VERSION 3.22)
set(PROJECT_VER "1.1.4")
# Private machine-local defaults (server address, home location...) live in
# sdkconfig.defaults.local, which is git-ignored. Use it when present.
if(NOT DEFINED SDKCONFIG_DEFAULTS AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.local")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.local")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(adsb_display)
