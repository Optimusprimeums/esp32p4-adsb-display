#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FEED_BEAST = 0,   /* binary Beast, usually port 30005 */
    FEED_AVR   = 1,   /* raw "*8D...;" hex lines, usually port 30002 */
    FEED_SBS   = 2,   /* SBS-1 / BaseStation CSV, usually port 30003 */
} feed_proto_t;

typedef enum {
    UNITS_AVIATION = 0,  /* ft, kt, nm, ft/min */
    UNITS_METRIC   = 1,  /* m, km/h, km, m/s */
} units_t;

typedef struct {
    char wifi_ssid[33];
    char wifi_pass[65];
    char host[64];
    uint16_t port;
    feed_proto_t proto;
    double rx_lat;
    double rx_lon;
    bool rx_pos_valid;
    bool online_lookup;
    units_t units;
    uint8_t brightness;      /* 5..100 % */
    uint16_t range_nm;       /* radar range */
    uint8_t vcom;            /* LCD panel VCOM register (image-retention tuning) */
    uint8_t tz;              /* index into the timezone table */
    bool flip;               /* screen rotated 180 degrees */
    bool saver;              /* screen saver: backlight off after inactivity */
    uint16_t saver_min;      /* screen saver timeout, minutes */
} app_settings_t;

void settings_init(void);
/* Returns a pointer to the live settings. Only the UI task mutates it. */
app_settings_t *settings_get(void);
void settings_save(void);
const char *feed_proto_name(feed_proto_t p);

/* Timezones offered in Settings (POSIX TZ rules, so DST switches automatically) */
int tz_count(void);
const char *tz_name(int i);
/* Newline-separated list for an LVGL dropdown */
const char *tz_dropdown_options(void);
/* Apply settings->tz to the C library (localtime) */
void settings_apply_tz(void);
