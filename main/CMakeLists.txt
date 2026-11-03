# The offline map is generated per location by tools/gen_map.py into
# map_data.c (git-ignored). Without one, build with an empty map.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/map_data.c")
    set(MAP_SRC "map_data.c")
else()
    set(MAP_SRC "map_data_none.c")
endif()

idf_component_register(
    SRCS
        "main.c"
        "board.c"
        "settings.c"
        "wifi.c"
        "adsb_client.c"
        "modes.c"
        "aircraft.c"
        "icao_db.c"
        "lookup.c"
        "ui.c"
        "selftest.c"
        "${MAP_SRC}"
        "ota.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_lcd esp_lcd_touch_gsl3680
        esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_hw_support
        esp_wifi esp_netif esp_event esp_timer nvs_flash lwip
        esp_http_client esp-tls mbedtls esp_app_format esp_psram
        app_update esp_http_server bootloader_support esp_system
)

# ESP-Hosted co-processor firmware for the ESP32-C6, pushed over SDIO on request
# (build it from c6_coprocessor/, then copy build/c6_hosted_coprocessor.bin to firmware/)
target_add_binary_data(${COMPONENT_LIB}
    "${CMAKE_CURRENT_LIST_DIR}/../c6_coprocessor/firmware/c6_hosted_coprocessor.bin" BINARY)
