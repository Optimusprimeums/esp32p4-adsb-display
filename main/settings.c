#include "settings.h"

#include "board.h"

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "settings";
static const char *NS = "adsb";

static app_settings_t s_cfg;

typedef struct {
    const char *name;
    const char *posix;
} tz_entry_t;

static const tz_entry_t s_tz[] = {
    {"UTC",                         "UTC0"},
    {"Hawaii (HST)",                "HST10"},
    {"Alaska",                      "AKST9AKDT,M3.2.0,M11.1.0"},
    {"US Pacific",                  "PST8PDT,M3.2.0,M11.1.0"},
    {"US Mountain",                 "MST7MDT,M3.2.0,M11.1.0"},
    {"Arizona",                     "MST7"},
    {"US Central",                  "CST6CDT,M3.2.0,M11.1.0"},
    {"US Eastern",                  "EST5EDT,M3.2.0,M11.1.0"},
    {"Atlantic",                    "AST4ADT,M3.2.0,M11.1.0"},
    {"UK / Ireland",                "GMT0BST,M3.5.0/1,M10.5.0"},
    {"Central Europe",              "CET-1CEST,M3.5.0,M10.5.0/3"},
    {"Eastern Europe",              "EET-2EEST,M3.5.0/3,M10.5.0/4"},
    {"Gulf (UAE)",                  "<+04>-4"},
    {"India",                       "IST-5:30"},
    {"China / Singapore / Perth",   "CST-8"},
    {"Japan / Korea",               "JST-9"},
    {"Australia East",              "AEST-10AEDT,M10.1.0,M4.1.0/3"},
    {"New Zealand",                 "NZST-12NZDT,M9.5.0,M4.1.0/3"},
};
#define TZ_DEFAULT CONFIG_ADSB_DEFAULT_TZ

int tz_count(void) { return sizeof(s_tz) / sizeof(s_tz[0]); }
const char *tz_name(int i) { return (i >= 0 && i < tz_count()) ? s_tz[i].name : "?"; }

const char *tz_dropdown_options(void)
{
    static char buf[512];
    if (!buf[0]) {
        for (int i = 0; i < tz_count(); i++) {
            if (i) strlcat(buf, "\n", sizeof(buf));
            strlcat(buf, s_tz[i].name, sizeof(buf));
        }
    }
    return buf;
}

void settings_apply_tz(void)
{
    int i = s_cfg.tz < tz_count() ? s_cfg.tz : 0;
    setenv("TZ", s_tz[i].posix, 1);
    tzset();
}

static void load_defaults(app_settings_t *c)
{
    memset(c, 0, sizeof(*c));
    strlcpy(c->wifi_ssid, CONFIG_ADSB_WIFI_SSID, sizeof(c->wifi_ssid));
    strlcpy(c->wifi_pass, CONFIG_ADSB_WIFI_PASSWORD, sizeof(c->wifi_pass));
    strlcpy(c->host, CONFIG_ADSB_SERVER_HOST, sizeof(c->host));
    c->port = CONFIG_ADSB_SERVER_PORT;
#if CONFIG_ADSB_PROTOCOL_AVR
    c->proto = FEED_AVR;
#elif CONFIG_ADSB_PROTOCOL_SBS
    c->proto = FEED_SBS;
#else
    c->proto = FEED_BEAST;
#endif
    c->rx_lat = strtod(CONFIG_ADSB_RX_LAT, NULL);
    c->rx_lon = strtod(CONFIG_ADSB_RX_LON, NULL);
    c->rx_pos_valid = !(c->rx_lat == 0.0 && c->rx_lon == 0.0);
#if CONFIG_ADSB_ONLINE_LOOKUP
    c->online_lookup = true;
#endif
    c->units = UNITS_AVIATION;
    c->brightness = 80;
    c->range_nm = 100;
    c->vcom = BOARD_VCOM_DEFAULT;
    c->tz = TZ_DEFAULT;
    c->flip = false;
    c->saver = true;
    c->saver_min = 10;
}

void settings_init(void)
{
    load_defaults(&s_cfg);
    settings_apply_tz();

    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored settings, using defaults");
        return;
    }
    size_t len;
    len = sizeof(s_cfg.wifi_ssid); nvs_get_str(h, "ssid", s_cfg.wifi_ssid, &len);
    len = sizeof(s_cfg.wifi_pass); nvs_get_str(h, "pass", s_cfg.wifi_pass, &len);
    len = sizeof(s_cfg.host);      nvs_get_str(h, "host", s_cfg.host, &len);
    nvs_get_u16(h, "port", &s_cfg.port);

    uint8_t u8;
    if (nvs_get_u8(h, "proto", &u8) == ESP_OK && u8 <= FEED_SBS) s_cfg.proto = (feed_proto_t)u8;
    if (nvs_get_u8(h, "units", &u8) == ESP_OK && u8 <= UNITS_METRIC) s_cfg.units = (units_t)u8;
    if (nvs_get_u8(h, "lookup", &u8) == ESP_OK) s_cfg.online_lookup = u8 != 0;
    if (nvs_get_u8(h, "bright", &u8) == ESP_OK && u8 >= 5 && u8 <= 100) s_cfg.brightness = u8;
    nvs_get_u16(h, "range", &s_cfg.range_nm);
    nvs_get_u8(h, "vcom", &s_cfg.vcom);
    if (nvs_get_u8(h, "tz", &u8) == ESP_OK && u8 < tz_count()) s_cfg.tz = u8;
    if (nvs_get_u8(h, "flip", &u8) == ESP_OK) s_cfg.flip = u8 != 0;
    if (nvs_get_u8(h, "saver", &u8) == ESP_OK) s_cfg.saver = u8 != 0;
    uint16_t u16;
    if (nvs_get_u16(h, "saver_min", &u16) == ESP_OK && u16 >= 1 && u16 <= 1440) s_cfg.saver_min = u16;

    /* doubles are stored as raw 64-bit blobs */
    int64_t raw;
    double lat = 0, lon = 0;
    if (nvs_get_i64(h, "lat", &raw) == ESP_OK) memcpy(&lat, &raw, sizeof(raw));
    if (nvs_get_i64(h, "lon", &raw) == ESP_OK) memcpy(&lon, &raw, sizeof(raw));
    /* a saved 0/0 means "never set": keep the built-in default location */
    if (lat != 0.0 || lon != 0.0) {
        s_cfg.rx_lat = lat;
        s_cfg.rx_lon = lon;
    }
    s_cfg.rx_pos_valid = !(s_cfg.rx_lat == 0.0 && s_cfg.rx_lon == 0.0);
    if (s_cfg.range_nm < 5 || s_cfg.range_nm > 400) s_cfg.range_nm = 100;

    nvs_close(h);
    settings_apply_tz();
    ESP_LOGI(TAG, "loaded: ssid='%s' host=%s:%u proto=%s rx=%.4f,%.4f",
             s_cfg.wifi_ssid, s_cfg.host, s_cfg.port, feed_proto_name(s_cfg.proto),
             s_cfg.rx_lat, s_cfg.rx_lon);
}

app_settings_t *settings_get(void)
{
    return &s_cfg;
}

void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return;
    }
    s_cfg.rx_pos_valid = !(s_cfg.rx_lat == 0.0 && s_cfg.rx_lon == 0.0);
    nvs_set_str(h, "ssid", s_cfg.wifi_ssid);
    nvs_set_str(h, "pass", s_cfg.wifi_pass);
    nvs_set_str(h, "host", s_cfg.host);
    nvs_set_u16(h, "port", s_cfg.port);
    nvs_set_u8(h, "proto", (uint8_t)s_cfg.proto);
    nvs_set_u8(h, "units", (uint8_t)s_cfg.units);
    nvs_set_u8(h, "lookup", s_cfg.online_lookup ? 1 : 0);
    nvs_set_u8(h, "bright", s_cfg.brightness);
    nvs_set_u16(h, "range", s_cfg.range_nm);
    nvs_set_u8(h, "vcom", s_cfg.vcom);
    nvs_set_u8(h, "tz", s_cfg.tz);
    nvs_set_u8(h, "flip", s_cfg.flip ? 1 : 0);
    nvs_set_u8(h, "saver", s_cfg.saver ? 1 : 0);
    nvs_set_u16(h, "saver_min", s_cfg.saver_min);
    int64_t raw;
    memcpy(&raw, &s_cfg.rx_lat, sizeof(raw)); nvs_set_i64(h, "lat", raw);
    memcpy(&raw, &s_cfg.rx_lon, sizeof(raw)); nvs_set_i64(h, "lon", raw);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "settings saved");
}

const char *feed_proto_name(feed_proto_t p)
{
    switch (p) {
    case FEED_BEAST: return "Beast";
    case FEED_AVR:   return "AVR";
    case FEED_SBS:   return "SBS";
    }
    return "?";
}
