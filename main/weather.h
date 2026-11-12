#pragma once

/*
 * Weather station page data, one JSON reading per line, from either
 *  - an RF controller's weather feed: raw TCP, it pushes "type":"reading" lines
 *    (e.g. port 30090), or
 *  - rtl_433's HTTP streaming API (rtl_433 ... -F http, GET /stream).
 * The format is detected from the first line, so only host and port are set.
 *
 * The client locks onto the first station it hears that reports wind or rain
 * (or, failing that, the first thermometer), so neighbours' sensors are
 * ignored. Field names follow rtl_433's conventions, so any rtl_433 weather
 * station works; tested target: Vevor / Fujian Youtong 7-in-1 (decoder 263).
 */

#include <stdbool.h>
#include <stdint.h>

#define WX_HIST_LEN     288        /* 24 h of 5-minute samples */
#define WX_HIST_STEP_S  300

typedef enum {
    WX_OFF = 0,          /* no server configured */
    WX_WAIT_WIFI,
    WX_CONNECTING,
    WX_CONNECTED,        /* connected, waiting for/receiving readings */
    WX_ERROR,
} wx_link_t;

typedef struct {
    float temp_c, hum, wind_kmh, gust_kmh, rain_total_mm;
    bool valid;
} wx_sample_t;

typedef struct {
    /* link */
    wx_link_t link;
    char detail[64];
    uint32_t msgs, ignored;

    /* station */
    bool locked;
    char model[32];
    char id[16];
    uint32_t last_ms;             /* now_ms() of the last reading, 0 = none */

    /* readings (metric) */
    bool temp_v, hum_v, wind_v, gust_v, dir_v, rain_v, uv_v, lux_v, press_v, batt_v, rssi_v;
    float temp_c, hum, wind_kmh, gust_kmh, dir_deg, rain_total_mm, uvi, lux, press_hpa, rssi_db;
    bool batt_ok;

    /* derived */
    bool dew_v, feels_v;
    float dew_c, feels_c;
    bool day_v;                   /* today's figures valid (needs the clock) */
    float rain_today_mm, rain_rate_mmh;
    float temp_hi, temp_lo, gust_max_kmh;

    /* 24 h history, oldest first when copied out */
    wx_sample_t hist[WX_HIST_LEN];
    uint16_t hist_n;
} wx_state_t;

void weather_start(void);
/* Drop the connection and reconnect with the current settings */
void weather_restart(void);
/* Thread-safe copy of the current state (large: keep it static or in PSRAM) */
void weather_get(wx_state_t *out);
const char *wx_link_str(wx_link_t l);
/* "N", "NNE", ... for a direction in degrees */
const char *wx_cardinal(float deg);
