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

typedef enum {
    WX_UNITS_METRIC = 0,     /* C, km/h, mm */
    WX_UNITS_IMPERIAL = 1,   /* F, mph, in */
    WX_UNITS_MARINE = 2,     /* C, kt, mm */
} wx_units_t;

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
    char wx_host[64];        /* weather feed: RF controller TCP or rtl_433 HTTP; empty = off */
    uint16_t wx_port;
    wx_units_t wx_units;
    char ab_host[64];        /* GopherTrunk HTTP API (airband audio); empty = off */
    uint16_t ab_port;
    bool ab_listen;          /* live audio on */
    uint8_t ab_volume;       /* 0..100 */
    bool ab_mute;
    char ab_filter[48];      /* GopherTrunk device_serial to play; empty = all channels */
    char ab_token[96];       /* GopherTrunk API bearer token for tuning; empty = none */
    char ab_presets[256];    /* "Label MHz; Label MHz" tuning presets */
    char ab_guard[128];      /* guarded preset frequencies, kHz, comma separated ("121500,118100") */
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
