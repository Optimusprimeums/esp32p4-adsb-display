cmake_minimum_required(VERSION 3.22)
set(PROJECT_VER "1.3.3")
# Private machine-local defaults (server address, home location...) live in
# sdkconfig.defaults.local, which is git-ignored. Use it when present.
# Build variants: idf.py -B build_<v> -D ADSB_VARIANT=<v> -D SDKCONFIG=build_<v>/sdkconfig build
# adds sdkconfig.defaults.<v> on top (wx: the weather view build).
if(NOT DEFINED SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.local")
        list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.local")
    endif()
    if(DEFINED ADSB_VARIANT)
        list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.${ADSB_VARIANT}")
    endif()
endif()
if(DEFINED ADSB_VARIANT)
    set(PROJECT_VER "${PROJECT_VER}-${ADSB_VARIANT}")
endif()
# ESP-Hosted is pinned to 3.0.9 to match the C6 firmware. The registry metadata
# of the newer 3.1.0 has a dependency rule this component manager rejects, and
# the "new versions available" check reads it, failing the configure step.
set(ENV{IDF_COMPONENT_CHECK_NEW_VERSION} 0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(adsb_display)
