#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#define WIFI_SCAN_MAX 20

typedef struct {
    char ssid[33];
    int8_t rssi;
    bool secure;
} wifi_ap_info_t;

/* Starts the ESP32-C6 link (esp_hosted) and the Wi-Fi station from settings */
void wifi_init(void);
/* Re-read SSID/password from settings and reconnect */
void wifi_apply_settings(void);

bool wifi_is_connected(void);
bool wifi_wait_connected(TickType_t timeout);
int  wifi_rssi(void);
void wifi_ip_str(char *buf, int len);
const char *wifi_state_str(void);

/* Async scan; poll wifi_scan_results() until it returns >= 0 */
void wifi_scan_start(void);
int  wifi_scan_results(wifi_ap_info_t *out, int max);  /* -1 while running */

bool time_is_synced(void);
