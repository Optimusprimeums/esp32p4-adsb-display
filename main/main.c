/*
 * ADS-B display for the Guition JC8012P4A1 (ESP32-P4 + ESP32-C6, 10.1" 800x1280).
 *
 * Connects to a dump1090 / readsb style server over TCP (Beast, raw AVR or
 * SBS), decodes Mode S / ADS-B / Comm-B itself, and shows a radar plot,
 * aircraft list and a detail page with everything that can be decoded or
 * looked up for each aircraft.
 */
#include "adsb_client.h"
#include "aircraft.h"
#include "board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lookup.h"
#include "ota.h"
#include "nvs_flash.h"
#include "settings.h"
#include "ui.h"
#include "wifi.h"

static const char *TAG = "main";

int modes_selftest(void);

static void lvgl_task(void *arg)
{
    lv_display_t *disp = board_init();
    ui_init(disp);
    for (;;) {
        uint32_t wait = lv_timer_handler();
        if (wait > 20) wait = 20;
        if (wait < 2) wait = 2;
        vTaskDelay(pdMS_TO_TICKS(wait));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ota_init();
    ota_report_selftest(modes_selftest() == 0);
    settings_init();
    aircraft_init();

    /* LVGL owns core 0's spare time; network work runs on core 1 */
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 16384, NULL, 4, NULL, 0);

    wifi_init();
    ota_start_server();
    adsb_client_start();
    lookup_start();
    ESP_LOGI(TAG, "started");
}
