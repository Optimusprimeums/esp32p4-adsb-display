#include "ui.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "adsb_client.h"
#include "aircraft.h"
#include "board.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lookup.h"
#include "map_data.h"
#include "ota.h"
#include "settings.h"
#include "src/misc/cache/instance/lv_image_cache.h"
#include "wifi.h"

static const char *TAG = "ui";

/* ------------------------------------------------------------------------- */
/* Layout                                                                    */
/* ------------------------------------------------------------------------- */

#define SCR_W        BOARD_LCD_H_RES
#define SCR_H        BOARD_LCD_V_RES
#define TOPBAR_H     56
#define RADAR_H      760
#define PANEL_Y      (TOPBAR_H + RADAR_H)
#define PANEL_H      (SCR_H - PANEL_Y)
#define RADAR_R      350          /* px radius of the outer range ring */
#define MAX_LIST     150
#define SNAP_TRAIL   AC_TRAIL_LEN /* trail points kept per aircraft for the radar */

#define COL_BG       lv_color_hex(0x0B1220)
#define COL_PANEL    lv_color_hex(0x131C2E)
#define COL_RING     lv_color_hex(0x2A3A55)
#define COL_TEXT     lv_color_hex(0xDCE6F5)
#define COL_DIM      lv_color_hex(0x7F8DA6)
#define COL_ACCENT   lv_color_hex(0x38BDF8)
#define COL_SEL      lv_color_hex(0xFACC15)
#define COL_EMERG    lv_color_hex(0xEF4444)
#define COL_OK       lv_color_hex(0x22C55E)
#define COL_WARN     lv_color_hex(0xF59E0B)

/* ------------------------------------------------------------------------- */
/* Snapshot of aircraft data for drawing (taken under the aircraft lock)     */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint32_t addr;
    char callsign[9];
    char reg[16];
    char type[8];
    bool pos; double lat, lon;
    bool alt_v; int32_t alt;
    bool ground;
    bool trk_v; float trk;
    bool gs_v; float gs;
    bool vr_v; int32_t vr;
    bool dist_v; double dist; float brg;
    bool sq_v; uint16_t squawk;
    bool emergency;
    bool military;
    uint32_t age_ms;
    uint16_t trail_n;
    float trail_lat[SNAP_TRAIL], trail_lon[SNAP_TRAIL];
    int32_t trail_alt[SNAP_TRAIL];
} snap_t;

static snap_t *s_snap;              /* AC_MAX entries, PSRAM */
static int s_snap_n;
static aircraft_t *s_sel_copy;      /* full copy of the selected aircraft */
static bool s_sel_valid;
static uint32_t s_selected;
static int s_with_pos;

/* radar state */
static double s_clat, s_clon;       /* map centre */
static bool s_center_valid;
static bool s_center_on_rx = true;
static bool s_show_labels = true;
static bool s_show_trails = true;
static bool s_show_map = true;
static bool s_dragged;

/* list state */
typedef enum { SORT_DIST, SORT_ALT, SORT_CALLSIGN, SORT_SPEED, SORT_RECENT } sort_t;
static sort_t s_sort = SORT_DIST;
static uint32_t s_row_addr[MAX_LIST + 1];
static int s_rows;

/* rate history for the stats chart */
#define RATE_HIST 120

/* widgets */
static lv_obj_t *s_radar, *s_range_lbl, *s_center_lbl;
static lv_obj_t *s_top_wifi, *s_top_feed, *s_top_count, *s_top_clock, *s_top_utc;
static lv_obj_t *s_banner, *s_banner_lbl;
static lv_obj_t *s_tabview;
static lv_obj_t *s_table;
static lv_obj_t *s_det_title, *s_det_sub1, *s_det_sub2, *s_det_route, *s_det_photo, *s_det_table, *s_det_empty;
static lv_obj_t *s_stats_lbl, *s_stats_chart, *s_df_lbl;
static lv_chart_series_t *s_rate_ser;
static lv_obj_t *s_kb;
static lv_obj_t *s_ta_ssid, *s_ta_pass, *s_ta_host, *s_ta_port, *s_ta_lat, *s_ta_lon;
static lv_obj_t *s_dd_proto, *s_dd_units, *s_sw_lookup, *s_sl_bright, *s_dd_scan, *s_btn_scan, *s_set_status;
static lv_obj_t *s_sl_vcom, *s_lbl_vcom, *s_dd_tz, *s_dd_flip, *s_sw_saver, *s_dd_saver;
static lv_obj_t *s_fw_lbl, *s_fw_status, *s_fw_bar, *s_btn_revert, *s_c6_lbl, *s_btn_c6;
static bool s_scan_pending;

static lv_image_dsc_t s_photo_dsc[2];
static int s_photo_idx;
static uint8_t *s_photo_buf[2];
static uint32_t s_photo_addr;

/* ------------------------------------------------------------------------- */
/* Formatting helpers                                                        */
/* ------------------------------------------------------------------------- */

static bool metric(void) { return settings_get()->units == UNITS_METRIC; }

static void fmt_alt(char *b, size_t n, int32_t ft)
{
    if (metric()) snprintf(b, n, "%ld m", (long)lroundf(ft * 0.3048f));
    else snprintf(b, n, "%ld ft", (long)ft);
}

static void fmt_alt_short(char *b, size_t n, const snap_t *s)
{
    if (s->ground) { snprintf(b, n, "GND"); return; }
    if (!s->alt_v) { snprintf(b, n, "-"); return; }
    if (metric()) snprintf(b, n, "%ld m", (long)lroundf(s->alt * 0.3048f));
    else if (s->alt >= 18000) snprintf(b, n, "FL%03ld", (long)(s->alt / 100));
    else snprintf(b, n, "%ld", (long)s->alt);
}

static void fmt_speed(char *b, size_t n, float kt)
{
    if (metric()) snprintf(b, n, "%d km/h", (int)lroundf(kt * 1.852f));
    else snprintf(b, n, "%d kt", (int)lroundf(kt));
}

static void fmt_dist(char *b, size_t n, double nm)
{
    if (metric()) snprintf(b, n, "%.1f km", nm * 1.852);
    else snprintf(b, n, "%.1f nm", nm);
}

static void fmt_vrate(char *b, size_t n, int32_t fpm)
{
    if (metric()) snprintf(b, n, "%+.1f m/s", fpm * 0.00508f);
    else snprintf(b, n, "%+ld ft/min", (long)fpm);
}

static const char *squawk_meaning(uint16_t sq)
{
    switch (sq) {
    case 0x7500: return "Hijack";
    case 0x7600: return "Radio failure";
    case 0x7700: return "Emergency";
    case 0x7400: return "UAV lost link";
    case 0x7000: return "VFR (ICAO)";
    case 0x1200: return "VFR (US)";
    case 0x2000: return "No code assigned";
    case 0x7777: return "Military intercept";
    case 0x0000: return "Military / no code";
    case 0x1000: return "Mode S conspicuity";
    default:     return NULL;
    }
}

static const char *nacp_str(uint8_t v)
{
    static const char *t[] = {"unknown", "<10 nm", "<4 nm", "<2 nm", "<1 nm", "<0.5 nm",
                              "<0.3 nm", "<0.1 nm", "<0.05 nm", "<30 m", "<10 m", "<3 m"};
    return v < 12 ? t[v] : "?";
}

static const char *nacv_str(uint8_t v)
{
    static const char *t[] = {"unknown", "<10 m/s", "<3 m/s", "<1 m/s", "<0.3 m/s"};
    return v < 5 ? t[v] : "?";
}

static const char *sil_str(uint8_t v)
{
    static const char *t[] = {"unknown", "1e-3", "1e-5", "1e-7"};
    return v < 4 ? t[v] : "?";
}

static const char *source_str(ac_source_t s)
{
    switch (s) {
    case SRC_ADSB:    return "ADS-B (DF17)";
    case SRC_ADSB_NT: return "ADS-B non-transponder";
    case SRC_ADSR:    return "ADS-R (UAT rebroadcast)";
    case SRC_TISB:    return "TIS-B (ground radar)";
    case SRC_SBS:     return "SBS feed";
    case SRC_MODE_S:  return "Mode S only";
    default:          return "-";
    }
}

static lv_color_t alt_color(bool alt_v, int32_t alt, bool ground)
{
    if (ground) return lv_color_hex(0x9CA3AF);
    if (!alt_v) return lv_color_hex(0xCBD5E1);
    float f = alt / 40000.0f;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    uint16_t hue = (uint16_t)(20 + f * 280);   /* orange -> green -> blue -> magenta */
    return lv_color_hsv_to_rgb(hue, 80, 98);
}

/* Same hue as alt_color(), slightly lighter and fully bright so trails stand out on the map */
static lv_color_t trail_color(int32_t alt)
{
    float f = alt / 40000.0f;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    return lv_color_hsv_to_rgb((uint16_t)(20 + f * 280), 65, 100);
}

static bool is_emergency_squawk(uint16_t sq)
{
    return sq == 0x7500 || sq == 0x7600 || sq == 0x7700;
}

/* ------------------------------------------------------------------------- */
/* Snapshot                                                                  */
/* ------------------------------------------------------------------------- */

static void take_snapshot(void)
{
    uint32_t t = now_ms();
    int n, out = 0;
    s_with_pos = 0;
    s_sel_valid = false;

    aircraft_lock();
    aircraft_t *tab = aircraft_table(&n);
    for (int i = 0; i < n; i++) {
        aircraft_t *a = &tab[i];
        if (!a->used) continue;
        snap_t *s = &s_snap[out++];
        s->addr = a->addr;
        strlcpy(s->callsign, a->callsign, sizeof(s->callsign));
        strlcpy(s->reg, a->registration, sizeof(s->reg));
        strlcpy(s->type, a->type_code, sizeof(s->type));
        s->pos = a->pos_valid;
        s->lat = a->lat;
        s->lon = a->lon;
        if (a->alt_baro_ms && t - a->alt_baro_ms < 60000) { s->alt_v = true; s->alt = a->alt_baro_ft; }
        else if (a->alt_geom_ms && t - a->alt_geom_ms < 60000) { s->alt_v = true; s->alt = a->alt_geom_ft; }
        else s->alt_v = false;
        s->ground = a->ground_valid && a->on_ground;
        s->trk_v = a->track_ms && t - a->track_ms < 60000;
        s->trk = a->track_deg;
        if (!s->trk_v && a->heading_ms && t - a->heading_ms < 60000) { s->trk_v = true; s->trk = a->heading_deg; }
        s->gs_v = a->gs_ms && t - a->gs_ms < 60000;
        s->gs = a->gs_kt;
        if (a->baro_rate_ms && t - a->baro_rate_ms < 60000) { s->vr_v = true; s->vr = a->baro_rate_fpm; }
        else if (a->geom_rate_ms && t - a->geom_rate_ms < 60000) { s->vr_v = true; s->vr = a->geom_rate_fpm; }
        else s->vr_v = false;
        s->dist_v = a->dist_valid;
        s->dist = a->dist_nm;
        s->brg = a->bearing_deg;
        s->sq_v = a->squawk_ms != 0;
        s->squawk = a->squawk;
        s->emergency = (a->emergency_valid && a->emergency != EMERG_NONE) ||
                       (s->sq_v && is_emergency_squawk(a->squawk));
        s->military = a->military;
        s->age_ms = t - a->last_seen_ms;
        if (s->pos) s_with_pos++;

        /* most recent trail points, oldest first */
        int keep = a->trail_count < SNAP_TRAIL ? a->trail_count : SNAP_TRAIL;
        if (!s_show_trails && a->addr != s_selected) keep = 0;
        for (int k = 0; k < keep; k++) {
            int idx = (a->trail_head + AC_TRAIL_LEN - keep + k) % AC_TRAIL_LEN;
            s->trail_lat[k] = a->trail[idx].lat;
            s->trail_lon[k] = a->trail[idx].lon;
            s->trail_alt[k] = a->trail[idx].alt_ft;
        }
        s->trail_n = keep;

        if (a->addr == s_selected) {
            memcpy(s_sel_copy, a, sizeof(aircraft_t));
            s_sel_valid = true;
        }
    }
    aircraft_unlock();
    s_snap_n = out;

    /* choose a map centre */
    const app_settings_t *cfg = settings_get();
    if (s_center_on_rx) {
        if (cfg->rx_pos_valid) {
            s_clat = cfg->rx_lat; s_clon = cfg->rx_lon; s_center_valid = true;
        } else if (s_with_pos) {
            double la = 0, lo = 0; int c = 0;
            for (int i = 0; i < s_snap_n; i++) if (s_snap[i].pos) { la += s_snap[i].lat; lo += s_snap[i].lon; c++; }
            s_clat = la / c; s_clon = lo / c; s_center_valid = true;
            s_center_on_rx = false;   /* freeze so the map doesn't wander */
        }
    }
}

static snap_t *snap_find(uint32_t addr)
{
    for (int i = 0; i < s_snap_n; i++) if (s_snap[i].addr == addr) return &s_snap[i];
    return NULL;
}

/* ------------------------------------------------------------------------- */
/* Radar                                                                     */
/* ------------------------------------------------------------------------- */

static float px_per_nm(void)
{
    return (float)RADAR_R / settings_get()->range_nm;
}

static void project(double lat, double lon, const lv_area_t *c, float *x, float *y)
{
    float s = px_per_nm();
    double dx = (lon - s_clon) * 60.0 * cos(s_clat * M_PI / 180.0);
    double dy = (lat - s_clat) * 60.0;
    *x = (c->x1 + c->x2) / 2.0f + (float)(dx * s);
    *y = (c->y1 + c->y2) / 2.0f - (float)(dy * s);
}

static void draw_text(lv_layer_t *layer, const char *txt, int32_t x, int32_t y, int32_t w,
                      lv_color_t col, const lv_font_t *font, lv_text_align_t align)
{
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.text = txt;
    d.text_local = 1;
    d.color = col;
    d.font = font;
    d.align = align;
    lv_area_t a = {x, y, x + w, y + lv_font_get_line_height(font) * 2};
    lv_draw_label(layer, &d, &a);
}

static void draw_line(lv_layer_t *layer, float x1, float y1, float x2, float y2,
                      lv_color_t col, int w, lv_opa_t opa)
{
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.p1.x = x1; d.p1.y = y1;
    d.p2.x = x2; d.p2.y = y2;
    d.color = col;
    d.width = w;
    d.opa = opa;
    d.round_start = d.round_end = 0;
    lv_draw_line(layer, &d);
}

static void draw_ring(lv_layer_t *layer, int32_t cx, int32_t cy, int32_t r, lv_color_t col, int w)
{
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.center.x = cx;
    d.center.y = cy;
    d.radius = r;
    d.start_angle = 0;
    d.end_angle = 360;
    d.width = w;
    d.color = col;
    lv_draw_arc(layer, &d);
}

static void draw_aircraft(lv_layer_t *layer, const snap_t *s, float x, float y, bool sel)
{
    lv_color_t col = alt_color(s->alt_v, s->alt, s->ground);
    if (s->emergency) col = COL_EMERG;

    if (s->trk_v) {
        float a = s->trk * (float)M_PI / 180.0f;
        float size = sel ? 16 : 12;
        lv_draw_triangle_dsc_t t;
        lv_draw_triangle_dsc_init(&t);
        t.p[0].x = x + sinf(a) * size;           t.p[0].y = y - cosf(a) * size;
        t.p[1].x = x + sinf(a + 2.5f) * size * 0.75f; t.p[1].y = y - cosf(a + 2.5f) * size * 0.75f;
        t.p[2].x = x + sinf(a - 2.5f) * size * 0.75f; t.p[2].y = y - cosf(a - 2.5f) * size * 0.75f;
        t.color = col;
        t.opa = LV_OPA_COVER;
        lv_draw_triangle(layer, &t);
    } else {
        lv_draw_rect_dsc_t r;
        lv_draw_rect_dsc_init(&r);
        r.bg_color = col;
        r.radius = LV_RADIUS_CIRCLE;
        lv_area_t a = {(int32_t)x - 5, (int32_t)y - 5, (int32_t)x + 5, (int32_t)y + 5};
        lv_draw_rect(layer, &r, &a);
    }

    if (sel) draw_ring(layer, (int32_t)x, (int32_t)y, 24, COL_SEL, 3);
    if (s->emergency) draw_ring(layer, (int32_t)x, (int32_t)y, 30, COL_EMERG, 3);
    if (s->military && !sel) draw_ring(layer, (int32_t)x, (int32_t)y, 18, lv_color_hex(0x84CC16), 1);

    if (s_show_labels || sel) {
        char txt[48], alt[16];
        fmt_alt_short(alt, sizeof(alt), s);
        const char *id = s->callsign[0] ? s->callsign : (s->reg[0] ? s->reg : NULL);
        char hex[8];
        if (!id) { snprintf(hex, sizeof(hex), "%06lX", (unsigned long)(s->addr & 0xFFFFFF)); id = hex; }
        snprintf(txt, sizeof(txt), "%s\n%s%s", id, alt,
                 s->vr_v ? (s->vr > 300 ? " " LV_SYMBOL_UP : s->vr < -300 ? " " LV_SYMBOL_DOWN : "") : "");
        draw_text(layer, txt, (int32_t)x + 14, (int32_t)y - 4, 140,
                  sel ? COL_SEL : COL_TEXT, &lv_font_montserrat_14, LV_TEXT_ALIGN_LEFT);
    }
}

/* Offline map layer: coastlines, runways, airport codes (data from tools/gen_map.py) */
static void draw_map(lv_layer_t *layer, const lv_area_t *c)
{
    const float range = settings_get()->range_nm;
    /* visible lat/lon window (with margin) for cheap culling */
    float half_w_nm = (c->x2 - c->x1) / 2.0f / px_per_nm() * 1.2f;
    float half_h_nm = (c->y2 - c->y1) / 2.0f / px_per_nm() * 1.2f;
    float lat_lo = s_clat - half_h_nm / 60.0f, lat_hi = s_clat + half_h_nm / 60.0f;
    float coslat = cosf(s_clat * (float)M_PI / 180.0f);
    float lon_lo = s_clon - half_w_nm / 60.0f / coslat, lon_hi = s_clon + half_w_nm / 60.0f / coslat;

    /* coastlines */
    const lv_color_t coast = lv_color_hex(0x3E7C8C);
    for (int i = 0; i < map_coast_line_count; i++) {
        const map_line_t *l = &map_coast_lines[i];
        if (l->lat_max < lat_lo || l->lat_min > lat_hi || l->lon_max < lon_lo || l->lon_min > lon_hi) continue;
        float px, py;
        project(map_coast_pts[l->start][0], map_coast_pts[l->start][1], c, &px, &py);
        for (uint32_t k = 1; k < l->count; k++) {
            float x, y;
            project(map_coast_pts[l->start + k][0], map_coast_pts[l->start + k][1], c, &x, &y);
            /* level of detail: skip points within 2 px of the last drawn one
             * (always keep the final point so lines join up) */
            if (k + 1 < l->count && fabsf(x - px) < 2.0f && fabsf(y - py) < 2.0f) continue;
            draw_line(layer, px, py, x, y, coast, 2, LV_OPA_COVER);
            px = x;
            py = y;
        }
    }

    /* runways: big airports always, small strips only when zoomed in */
    const lv_color_t rwy = lv_color_hex(0xA9B6C8);
    for (int i = 0; i < map_runway_count; i++) {
        const map_runway_t *r = &map_runways[i];
        if (r->kind == 2 && range > 40) continue;
        if (r->lat1 < lat_lo || r->lat1 > lat_hi || r->lon1 < lon_lo || r->lon1 > lon_hi) continue;
        float x1, y1, x2, y2;
        project(r->lat1, r->lon1, c, &x1, &y1);
        project(r->lat2, r->lon2, c, &x2, &y2);
        draw_line(layer, x1, y1, x2, y2, rwy, range <= 40 ? 4 : 2, LV_OPA_COVER);
    }

    if (map_coast_credit[0]) {
        draw_text(layer, map_coast_credit, c->x1 + 12, c->y1 + 30, 200, COL_DIM,
                  &lv_font_montserrat_12, LV_TEXT_ALIGN_LEFT);
    }

    /* airport codes */
    for (int i = 0; i < map_airport_count; i++) {
        const map_airport_t *ap = &map_airports[i];
        if (ap->kind == 1 && range > 120) continue;
        if (ap->kind == 2 && range > 40) continue;
        if (ap->lat < lat_lo || ap->lat > lat_hi || ap->lon < lon_lo || ap->lon > lon_hi) continue;
        float x, y;
        project(ap->lat, ap->lon, c, &x, &y);
        const char *code = ap->iata[0] ? ap->iata : ap->code;
        draw_text(layer, code, (int32_t)x - 40, (int32_t)y + 8, 80,
                  ap->kind == 0 ? lv_color_hex(0xC7D2E3) : COL_DIM,
                  ap->kind == 0 ? &lv_font_montserrat_14 : &lv_font_montserrat_12, LV_TEXT_ALIGN_CENTER);
    }
}

static void radar_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t c;
    lv_obj_get_coords(obj, &c);
    int32_t cx = (c.x1 + c.x2) / 2, cy = (c.y1 + c.y2) / 2;
    const app_settings_t *cfg = settings_get();

    /* receiver position relative to map centre */
    float rx = cx, ry = cy;
    if (cfg->rx_pos_valid && s_center_valid) project(cfg->rx_lat, cfg->rx_lon, &c, &rx, &ry);

    if (s_center_valid && s_show_map) draw_map(layer, &c);

    /* range rings around the receiver */
    for (int i = 1; i <= 4; i++) {
        int32_t r = RADAR_R * i / 4;
        draw_ring(layer, (int32_t)rx, (int32_t)ry, r, COL_RING, i == 4 ? 2 : 1);
        char lbl[16];
        double nm = cfg->range_nm * i / 4.0;
        if (metric()) snprintf(lbl, sizeof(lbl), "%.0f km", nm * 1.852);
        else snprintf(lbl, sizeof(lbl), "%.0f nm", nm);
        draw_text(layer, lbl, (int32_t)rx + 4, (int32_t)ry - r - 18, 80, COL_DIM,
                  &lv_font_montserrat_12, LV_TEXT_ALIGN_LEFT);
    }
    draw_line(layer, rx - RADAR_R, ry, rx + RADAR_R, ry, COL_RING, 1, LV_OPA_60);
    draw_line(layer, rx, ry - RADAR_R, rx, ry + RADAR_R, COL_RING, 1, LV_OPA_60);
    static const char *card[] = {"N", "E", "S", "W"};
    for (int i = 0; i < 4; i++) {
        float a = i * (float)M_PI / 2;
        draw_text(layer, card[i], (int32_t)(rx + sinf(a) * (RADAR_R + 14)) - 10,
                  (int32_t)(ry - cosf(a) * (RADAR_R + 14)) - 9, 20, COL_DIM,
                  &lv_font_montserrat_16, LV_TEXT_ALIGN_CENTER);
    }
    if (cfg->rx_pos_valid) {
        lv_draw_rect_dsc_t r;
        lv_draw_rect_dsc_init(&r);
        r.bg_color = COL_ACCENT;
        r.radius = 2;
        lv_area_t a = {(int32_t)rx - 4, (int32_t)ry - 4, (int32_t)rx + 4, (int32_t)ry + 4};
        lv_draw_rect(layer, &r, &a);
    }

    if (!s_center_valid) {
        draw_text(layer, "Waiting for aircraft positions...\nSet the receiver location in Settings.",
                  cx - 250, cy + 40, 500, COL_DIM, &lv_font_montserrat_20, LV_TEXT_ALIGN_CENTER);
        return;
    }

    /* trails: altitude-coloured, fading out with age (oldest point first) */
    int budget = 10000;
    for (int i = 0; i < s_snap_n && budget > 0; i++) {
        const snap_t *s = &s_snap[i];
        bool sel = s->addr == s_selected;
        for (int k = 1; k < s->trail_n && budget > 0; k++, budget--) {
            float x1, y1, x2, y2;
            project(s->trail_lat[k - 1], s->trail_lon[k - 1], &c, &x1, &y1);
            project(s->trail_lat[k], s->trail_lon[k], &c, &x2, &y2);
            if ((x1 < c.x1 && x2 < c.x1) || (x1 > c.x2 && x2 > c.x2) ||
                (y1 < c.y1 && y2 < c.y1) || (y1 > c.y2 && y2 > c.y2)) continue;
            lv_opa_t opa = sel ? LV_OPA_COVER : (lv_opa_t)(LV_OPA_40 + (LV_OPA_COVER - LV_OPA_40) * k / s->trail_n);
            lv_color_t col = sel ? alt_color(true, s->trail_alt[k], false) : trail_color(s->trail_alt[k]);
            draw_line(layer, x1, y1, x2, y2, col, sel ? 4 : 2, opa);
        }
    }

    /* aircraft (selected last so it's on top) */
    const snap_t *sel_s = NULL;
    for (int i = 0; i < s_snap_n; i++) {
        const snap_t *s = &s_snap[i];
        if (!s->pos) continue;
        if (s->addr == s_selected) { sel_s = s; continue; }
        float x, y;
        project(s->lat, s->lon, &c, &x, &y);
        if (x < c.x1 - 20 || x > c.x2 + 20 || y < c.y1 - 20 || y > c.y2 + 20) continue;
        draw_aircraft(layer, s, x, y, false);
    }
    if (sel_s) {
        float x, y;
        project(sel_s->lat, sel_s->lon, &c, &x, &y);
        draw_aircraft(layer, sel_s, x, y, true);
    }
}

static void select_aircraft(uint32_t addr, bool show_details);

static void radar_input_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;

    if (code == LV_EVENT_PRESSED) {
        s_dragged = false;
    } else if (code == LV_EVENT_PRESSING) {
        lv_point_t v;
        lv_indev_get_vect(indev, &v);
        if (v.x || v.y) {
            if (abs(v.x) + abs(v.y) > 2) s_dragged = true;
            if (s_dragged && s_center_valid) {
                float s = px_per_nm();
                s_clat += (v.y / s) / 60.0;
                s_clon -= (v.x / s) / 60.0 / cos(s_clat * M_PI / 180.0);
                s_center_on_rx = false;
                lv_obj_invalidate(s_radar);
            }
        }
    } else if (code == LV_EVENT_CLICKED && !s_dragged) {
        lv_point_t p;
        lv_indev_get_point(indev, &p);
        lv_area_t c;
        lv_obj_get_coords(s_radar, &c);
        float best = 45 * 45;
        uint32_t hit = 0;
        for (int i = 0; i < s_snap_n; i++) {
            if (!s_snap[i].pos) continue;
            float x, y;
            project(s_snap[i].lat, s_snap[i].lon, &c, &x, &y);
            float d = (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y);
            if (d < best) { best = d; hit = s_snap[i].addr; }
        }
        if (hit) {
            select_aircraft(hit, true);
        } else if (s_selected) {
            /* tap on empty map: clear the selection, stay on the current tab */
            select_aircraft(0, false);
        }
    }
}

static void update_range_label(void)
{
    char b[32];
    double nm = settings_get()->range_nm;
    if (metric()) snprintf(b, sizeof(b), "Range %.0f km", nm * 1.852);
    else snprintf(b, sizeof(b), "Range %.0f nm", nm);
    lv_label_set_text(s_range_lbl, b);
}

static void zoom_cb(lv_event_t *e)
{
    static const uint16_t steps[] = {5, 10, 15, 25, 40, 60, 80, 100, 150, 200, 250, 300, 400};
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    app_settings_t *cfg = settings_get();
    int idx = 0;
    for (int i = 0; i < (int)(sizeof(steps) / sizeof(steps[0])); i++) if (steps[i] <= cfg->range_nm) idx = i;
    idx += dir;
    if (idx < 0) idx = 0;
    if (idx >= (int)(sizeof(steps) / sizeof(steps[0]))) idx = sizeof(steps) / sizeof(steps[0]) - 1;
    cfg->range_nm = steps[idx];
    update_range_label();
    lv_obj_invalidate(s_radar);
}

static void recenter_cb(lv_event_t *e)
{
    s_center_on_rx = true;
    s_center_valid = false;
    take_snapshot();
    lv_obj_invalidate(s_radar);
}

static void toggle_cb(lv_event_t *e)
{
    bool *flag = lv_event_get_user_data(e);
    *flag = !*flag;
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_opa(btn, *flag ? LV_OPA_COVER : LV_OPA_40, 0);
    lv_obj_invalidate(s_radar);
}

static lv_obj_t *radar_btn(lv_obj_t *parent, const char *txt, lv_align_t al, int32_t x, int32_t y,
                           lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_set_size(b, 64, 56);
    lv_obj_align(b, al, x, y);
    lv_obj_set_style_bg_color(b, COL_PANEL, 0);
    lv_obj_set_style_border_color(b, COL_RING, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_24, 0);
    lv_obj_center(l);
    return b;
}

static void build_radar(lv_obj_t *scr)
{
    s_radar = lv_obj_create(scr);
    lv_obj_remove_style_all(s_radar);
    lv_obj_set_pos(s_radar, 0, TOPBAR_H);
    lv_obj_set_size(s_radar, SCR_W, RADAR_H);
    lv_obj_set_style_bg_color(s_radar, COL_BG, 0);
    lv_obj_set_style_bg_opa(s_radar, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_radar, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(s_radar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_radar, radar_draw_cb, LV_EVENT_DRAW_MAIN_END, NULL);
    lv_obj_add_event_cb(s_radar, radar_input_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s_radar, radar_input_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(s_radar, radar_input_cb, LV_EVENT_CLICKED, NULL);

    radar_btn(s_radar, LV_SYMBOL_PLUS, LV_ALIGN_BOTTOM_RIGHT, -12, -12, zoom_cb, (void *)(intptr_t)-1);
    radar_btn(s_radar, LV_SYMBOL_MINUS, LV_ALIGN_BOTTOM_RIGHT, -12, -76, zoom_cb, (void *)(intptr_t)1);
    radar_btn(s_radar, LV_SYMBOL_GPS, LV_ALIGN_BOTTOM_RIGHT, -12, -140, recenter_cb, NULL);
    lv_obj_t *b1 = radar_btn(s_radar, "Aa", LV_ALIGN_BOTTOM_LEFT, 12, -12, toggle_cb, &s_show_labels);
    lv_obj_t *b2 = radar_btn(s_radar, LV_SYMBOL_SHUFFLE, LV_ALIGN_BOTTOM_LEFT, 12, -76, toggle_cb, &s_show_trails);
    lv_obj_t *b3 = radar_btn(s_radar, LV_SYMBOL_IMAGE, LV_ALIGN_BOTTOM_LEFT, 12, -140, toggle_cb, &s_show_map);
    lv_obj_set_style_bg_color(b1, COL_PANEL, 0);
    lv_obj_set_style_bg_color(b2, COL_PANEL, 0);
    lv_obj_set_style_bg_color(b3, COL_PANEL, 0);

    s_range_lbl = lv_label_create(s_radar);
    lv_obj_set_style_text_color(s_range_lbl, COL_DIM, 0);
    lv_obj_align(s_range_lbl, LV_ALIGN_TOP_RIGHT, -12, 10);
    update_range_label();

    s_center_lbl = lv_label_create(s_radar);
    lv_obj_set_style_text_color(s_center_lbl, COL_DIM, 0);
    lv_obj_set_style_text_font(s_center_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(s_center_lbl, LV_ALIGN_TOP_LEFT, 12, 10);

    /* altitude colour legend */
    lv_obj_t *leg = lv_obj_create(s_radar);
    lv_obj_remove_style_all(leg);
    lv_obj_set_size(leg, 300, 22);
    lv_obj_align(leg, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_set_flex_flow(leg, LV_FLEX_FLOW_ROW);
    static const int32_t alts[] = {0, 5000, 10000, 20000, 30000, 40000};
    static const char *names[] = {"0", "5k", "10k", "20k", "30k", "40k+"};
    for (int i = 0; i < 6; i++) {
        lv_obj_t *l = lv_label_create(leg);
        lv_label_set_text(l, names[i]);
        lv_obj_set_width(l, 50);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(l, lv_color_black(), 0);
        lv_obj_set_style_bg_color(l, alt_color(true, alts[i], false), 0);
        lv_obj_set_style_bg_opa(l, LV_OPA_COVER, 0);
    }
}

/* ------------------------------------------------------------------------- */
/* Top bar + emergency banner                                                */
/* ------------------------------------------------------------------------- */

static void build_topbar(lv_obj_t *scr)
{
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, SCR_W, TOPBAR_H);
    lv_obj_set_style_bg_color(bar, COL_PANEL, 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(bar, 12, 0);
    lv_obj_remove_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    s_top_wifi = lv_label_create(bar);
    lv_obj_align(s_top_wifi, LV_ALIGN_LEFT_MID, 0, -10);
    lv_obj_set_style_text_font(s_top_wifi, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_top_wifi, COL_TEXT, 0);

    s_top_feed = lv_label_create(bar);
    lv_obj_align(s_top_feed, LV_ALIGN_LEFT_MID, 0, 12);
    lv_obj_set_style_text_font(s_top_feed, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_top_feed, COL_DIM, 0);
    lv_obj_set_width(s_top_feed, 420);
    lv_label_set_long_mode(s_top_feed, LV_LABEL_LONG_MODE_DOTS);

    s_top_count = lv_label_create(bar);
    lv_obj_align(s_top_count, LV_ALIGN_CENTER, 110, 0);
    lv_obj_set_style_text_font(s_top_count, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_top_count, COL_TEXT, 0);

    s_top_clock = lv_label_create(bar);
    lv_obj_align(s_top_clock, LV_ALIGN_TOP_RIGHT, 0, 4);
    lv_obj_set_style_text_font(s_top_clock, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_top_clock, COL_ACCENT, 0);

    s_top_utc = lv_label_create(bar);
    lv_obj_align(s_top_utc, LV_ALIGN_BOTTOM_RIGHT, 0, -2);
    lv_obj_set_style_text_font(s_top_utc, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_top_utc, COL_DIM, 0);
    lv_label_set_text(s_top_utc, "");
}

static void banner_cb(lv_event_t *e)
{
    uint32_t addr = (uint32_t)(uintptr_t)lv_obj_get_user_data(s_banner);
    if (addr) select_aircraft(addr, true);
}

static void build_banner(lv_obj_t *scr)
{
    s_banner = lv_button_create(scr);
    lv_obj_set_size(s_banner, SCR_W - 160, 52);
    lv_obj_align(s_banner, LV_ALIGN_TOP_MID, 0, TOPBAR_H + 44);
    lv_obj_set_style_bg_color(s_banner, COL_EMERG, 0);
    lv_obj_set_style_radius(s_banner, 10, 0);
    lv_obj_add_event_cb(s_banner, banner_cb, LV_EVENT_CLICKED, NULL);
    s_banner_lbl = lv_label_create(s_banner);
    lv_obj_set_style_text_font(s_banner_lbl, &lv_font_montserrat_20, 0);
    lv_obj_center(s_banner_lbl);
    lv_obj_add_flag(s_banner, LV_OBJ_FLAG_HIDDEN);
}

/* ------------------------------------------------------------------------- */
/* Aircraft list                                                             */
/* ------------------------------------------------------------------------- */

static int cmp_snap(const void *pa, const void *pb)
{
    const snap_t *a = *(const snap_t **)pa, *b = *(const snap_t **)pb;
    if (a->emergency != b->emergency) return a->emergency ? -1 : 1;
    switch (s_sort) {
    case SORT_DIST:
        if (a->dist_v != b->dist_v) return a->dist_v ? -1 : 1;
        return a->dist < b->dist ? -1 : a->dist > b->dist;
    case SORT_ALT: {
        int32_t aa = a->ground ? -1000 : (a->alt_v ? a->alt : -2000);
        int32_t bb = b->ground ? -1000 : (b->alt_v ? b->alt : -2000);
        return bb - aa;
    }
    case SORT_CALLSIGN: {
        const char *x = a->callsign[0] ? a->callsign : "~";
        const char *y = b->callsign[0] ? b->callsign : "~";
        return strcmp(x, y);
    }
    case SORT_SPEED:
        return (b->gs_v ? b->gs : -1) > (a->gs_v ? a->gs : -1) ? 1 : -1;
    case SORT_RECENT:
        return a->age_ms < b->age_ms ? -1 : a->age_ms > b->age_ms;
    }
    return 0;
}

static void update_list(void)
{
    static const snap_t *order[AC_MAX];
    int n = 0;
    for (int i = 0; i < s_snap_n; i++) order[n++] = &s_snap[i];
    qsort(order, n, sizeof(order[0]), cmp_snap);
    if (n > MAX_LIST) n = MAX_LIST;

    lv_table_set_row_count(s_table, n + 1);
    s_rows = n;
    char b[32];
    for (int i = 0; i < n; i++) {
        const snap_t *s = order[i];
        int r = i + 1;
        s_row_addr[r] = s->addr;
        lv_table_set_cell_value(s_table, r, 0, s->callsign[0] ? s->callsign : "-");
        if (s->reg[0]) lv_table_set_cell_value(s_table, r, 1, s->reg);
        else {
            snprintf(b, sizeof(b), "%s%06lX", (s->addr & 0x1000000) ? "~" : "", (unsigned long)(s->addr & 0xFFFFFF));
            lv_table_set_cell_value(s_table, r, 1, b);
        }
        lv_table_set_cell_value(s_table, r, 2, s->type[0] ? s->type : "");
        fmt_alt_short(b, sizeof(b), s);
        lv_table_set_cell_value(s_table, r, 3, b);
        if (s->gs_v) snprintf(b, sizeof(b), "%d", (int)lroundf(metric() ? s->gs * 1.852f : s->gs));
        else strcpy(b, "-");
        lv_table_set_cell_value(s_table, r, 4, b);
        if (s->vr_v) snprintf(b, sizeof(b), "%+ld", (long)(metric() ? lroundf(s->vr * 0.00508f) : s->vr));
        else strcpy(b, "");
        lv_table_set_cell_value(s_table, r, 5, b);
        if (s->dist_v) snprintf(b, sizeof(b), "%.0f", metric() ? s->dist * 1.852 : s->dist);
        else strcpy(b, "-");
        lv_table_set_cell_value(s_table, r, 6, b);
        if (s->sq_v) snprintf(b, sizeof(b), "%04X", s->squawk);
        else strcpy(b, "");
        lv_table_set_cell_value(s_table, r, 7, b);
    }
    lv_table_set_cell_value(s_table, 0, 4, metric() ? "km/h" : "GS kt");
    lv_table_set_cell_value(s_table, 0, 5, metric() ? "VS m/s" : "VS fpm");
    lv_table_set_cell_value(s_table, 0, 6, metric() ? "km" : "nm");
}

static void table_draw_cb(lv_event_t *e)
{
    lv_draw_task_t *t = lv_event_get_draw_task(e);
    lv_draw_dsc_base_t *base = lv_draw_task_get_draw_dsc(t);
    if (base->part != LV_PART_ITEMS) return;
    uint32_t row = base->id1;
    if (row == 0) {
        lv_draw_fill_dsc_t *f = lv_draw_task_get_fill_dsc(t);
        if (f) f->color = lv_color_hex(0x1E293B);
        lv_draw_label_dsc_t *l = lv_draw_task_get_label_dsc(t);
        if (l) l->color = COL_ACCENT;
        return;
    }
    if (row > (uint32_t)s_rows) return;
    uint32_t addr = s_row_addr[row];
    snap_t *s = snap_find(addr);
    lv_draw_fill_dsc_t *f = lv_draw_task_get_fill_dsc(t);
    if (f) {
        if (addr == s_selected) f->color = lv_color_hex(0x3B3412);
        else if (s && s->emergency) f->color = lv_color_hex(0x5B1A1A);
        else f->color = (row & 1) ? COL_PANEL : lv_color_hex(0x0F172A);
    }
    lv_draw_label_dsc_t *l = lv_draw_task_get_label_dsc(t);
    if (l && s) {
        if (base->id2 == 3) l->color = alt_color(s->alt_v, s->alt, s->ground);
        else if (s->military && base->id2 == 0) l->color = lv_color_hex(0x84CC16);
        else if (s->age_ms > 20000) l->color = COL_DIM;
    }
}

static void table_click_cb(lv_event_t *e)
{
    uint32_t row, col;
    lv_table_get_selected_cell(s_table, &row, &col);
    if (row == LV_TABLE_CELL_NONE || row == 0 || row > (uint32_t)s_rows) return;
    select_aircraft(s_row_addr[row], true);
}

static void sort_cb(lv_event_t *e)
{
    lv_obj_t *bm = lv_event_get_target(e);
    s_sort = (sort_t)lv_buttonmatrix_get_selected_button(bm);
    update_list();
}

static void build_list_tab(lv_obj_t *tab)
{
    lv_obj_set_style_pad_all(tab, 6, 0);
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);

    static const char *map[] = {"Distance", "Altitude", "Callsign", "Speed", "Recent", ""};
    lv_obj_t *bm = lv_buttonmatrix_create(tab);
    lv_buttonmatrix_set_map(bm, map);
    lv_buttonmatrix_set_button_ctrl_all(bm, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(bm, true);
    lv_buttonmatrix_set_button_ctrl(bm, 0, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_obj_set_size(bm, lv_pct(100), 48);
    lv_obj_set_style_pad_all(bm, 2, 0);
    lv_obj_add_event_cb(bm, sort_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s_table = lv_table_create(tab);
    lv_obj_set_width(s_table, lv_pct(100));
    lv_obj_set_flex_grow(s_table, 1);
    lv_table_set_column_count(s_table, 8);
    static const int32_t w[] = {130, 120, 70, 100, 80, 100, 80, 80};
    static const char *hdr[] = {"Flight", "Reg / Hex", "Type", "Alt", "GS kt", "VS fpm", "nm", "Sqk"};
    for (int i = 0; i < 8; i++) {
        lv_table_set_column_width(s_table, i, w[i]);
        lv_table_set_cell_value(s_table, 0, i, hdr[i]);
    }
    lv_obj_set_style_pad_ver(s_table, 6, LV_PART_ITEMS);
    lv_obj_set_style_pad_hor(s_table, 4, LV_PART_ITEMS);
    lv_obj_set_style_text_font(s_table, &lv_font_montserrat_16, LV_PART_ITEMS);
    lv_obj_set_style_border_width(s_table, 0, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(s_table, COL_PANEL, 0);
    lv_obj_add_flag(s_table, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);
    lv_obj_add_event_cb(s_table, table_draw_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
    lv_obj_add_event_cb(s_table, table_click_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

/* ------------------------------------------------------------------------- */
/* Details                                                                   */
/* ------------------------------------------------------------------------- */

typedef struct {
    const char *name;    /* NULL name + text => section header */
    char value[160];
    bool header;
} det_row_t;

#define DET_MAX_ROWS 90
static det_row_t s_det_rows[DET_MAX_ROWS];
static int s_det_n;

static void det_hdr(const char *t)
{
    if (s_det_n >= DET_MAX_ROWS) return;
    det_row_t *r = &s_det_rows[s_det_n++];
    r->name = t;
    r->value[0] = 0;
    r->header = true;
}

static void det_row(const char *name, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void det_row(const char *name, const char *fmt, ...)
{
    if (s_det_n >= DET_MAX_ROWS) return;
    det_row_t *r = &s_det_rows[s_det_n++];
    r->name = name;
    r->header = false;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(r->value, sizeof(r->value), fmt, ap);
    va_end(ap);
}

static const char *age_str(uint32_t t, uint32_t ms, char *b, size_t n)
{
    if (!ms) { snprintf(b, n, "never"); return b; }
    uint32_t a = (t - ms) / 1000;
    if (a < 2) snprintf(b, n, "now");
    else if (a < 120) snprintf(b, n, "%lus ago", (unsigned long)a);
    else snprintf(b, n, "%lum ago", (unsigned long)(a / 60));
    return b;
}

/* "value" if fresh, else "value (old)" */
#define FRESH(ms) ((ms) && (t - (ms)) < 30000)

static void fmt_airport(char *b, size_t n, const airport_t *ap)
{
    snprintf(b, n, "%s%s%s %s%s%s", ap->iata[0] ? ap->iata : "", ap->iata[0] ? " / " : "", ap->icao,
             ap->city, ap->country[0] ? ", " : "", ap->country);
}

static void build_details_rows(const aircraft_t *a)
{
    uint32_t t = now_ms();
    char b[64], c[160];
    s_det_n = 0;

    det_hdr("IDENTITY");
    det_row("ICAO address", "%s%06lX%s", (a->addr & 0x1000000) ? "~" : "", (unsigned long)(a->addr & 0xFFFFFF),
            (a->addr & 0x1000000) ? " (non-ICAO)" : "");
    det_row("Callsign", "%s", a->callsign[0] ? a->callsign : "-");
    det_row("Registration", "%s", a->registration[0] ? a->registration : "-");
    det_row("Type", "%s%s%s", a->type_code, a->type_code[0] && a->type_name[0] ? "  " : "",
            a->type_name[0] ? a->type_name : (a->type_code[0] ? "" : "-"));
    det_row("Manufacturer", "%s", a->manufacturer[0] ? a->manufacturer : "-");
    det_row("Owner / operator", "%s", a->owner[0] ? a->owner : "-");
    det_row("Registered in", "%s", a->owner_country[0] ? a->owner_country : "-");
    det_row("Address block", "%s", a->country ? a->country : "-");
    det_row("Category", "%s", a->category_valid ? category_str(a->category) : "-");
    det_row("Military", "%s", a->military ? "Likely (military address range)" : "No");
    det_row("Data source", "%s", source_str(a->source));
    if (a->adsb_version_valid) det_row("ADS-B version", "%u%s", a->adsb_version,
                                       a->adsb_version == 2 ? " (DO-260B)" : a->adsb_version == 1 ? " (DO-260A)" : " (DO-260)");
    else det_row("ADS-B version", "-");
    if (a->squawk_ms) {
        const char *m = squawk_meaning(a->squawk);
        det_row("Squawk", "%04X%s%s", a->squawk, m ? "  " : "", m ? m : "");
    } else {
        det_row("Squawk", "-");
    }

    det_hdr("ROUTE");
    if (a->lookup == LOOKUP_PENDING) det_row("Lookup", "in progress...");
    else if (a->lookup == LOOKUP_FAILED) det_row("Lookup", "failed (will retry)");
    else if (!settings_get()->online_lookup) det_row("Lookup", "disabled in settings");
    if (a->airline[0]) det_row("Airline", "%s%s%s%s", a->airline, a->airline_callsign[0] ? " (\"" : "",
                               a->airline_callsign, a->airline_callsign[0] ? "\")" : "");
    if (a->origin.valid) {
        fmt_airport(c, sizeof(c), &a->origin);
        det_row("From", "%s", c);
        det_row("", "%s", a->origin.name);
    }
    if (a->destination.valid) {
        fmt_airport(c, sizeof(c), &a->destination);
        det_row("To", "%s", c);
        det_row("", "%s", a->destination.name);
    }
    if (a->pos_valid && a->origin.valid && a->destination.valid) {
        double d_from = 2 * 3440.065 * asin(sqrt(pow(sin((a->lat - a->origin.lat) * M_PI / 360), 2) +
                        cos(a->lat * M_PI / 180) * cos(a->origin.lat * M_PI / 180) * pow(sin((a->lon - a->origin.lon) * M_PI / 360), 2)));
        double d_to = 2 * 3440.065 * asin(sqrt(pow(sin((a->lat - a->destination.lat) * M_PI / 360), 2) +
                      cos(a->lat * M_PI / 180) * cos(a->destination.lat * M_PI / 180) * pow(sin((a->lon - a->destination.lon) * M_PI / 360), 2)));
        fmt_dist(b, sizeof(b), d_from);
        fmt_dist(c, sizeof(c), d_to);
        det_row("Flown / remaining", "%s / %s", b, c);
        if (a->gs_ms && a->gs_kt > 50 && !a->on_ground) {
            int mins = (int)(d_to / a->gs_kt * 60.0);
            det_row("ETE at current GS", "%dh %02dm", mins / 60, mins % 60);
        }
    }
    if (!a->airline[0] && !a->origin.valid && a->lookup == LOOKUP_NOT_FOUND) det_row("Route", "unknown");

    det_hdr("POSITION");
    if (a->pos_valid) {
        det_row("Latitude", "%.5f", a->lat);
        det_row("Longitude", "%.5f", a->lon);
        det_row("Position age", "%s", age_str(t, a->pos_ms, b, sizeof(b)));
    } else {
        det_row("Position", "unknown (no CPR pair yet / Mode S only)");
    }
    if (a->dist_valid) {
        fmt_dist(b, sizeof(b), a->dist_nm);
        det_row("From receiver", "%s at %03.0f\xC2\xB0", b, a->bearing_deg);
    }
    if (a->alt_baro_ms) {
        fmt_alt(b, sizeof(b), a->alt_baro_ft);
        det_row("Baro altitude", "%s  (FL%03ld)%s", b, (long)(a->alt_baro_ft / 100), FRESH(a->alt_baro_ms) ? "" : " old");
    } else det_row("Baro altitude", "-");
    if (a->alt_geom_ms) {
        fmt_alt(b, sizeof(b), a->alt_geom_ft);
        det_row("GNSS altitude", "%s%s", b, FRESH(a->alt_geom_ms) ? "" : " old");
    }
    if (a->gnss_baro_diff_valid) {
        fmt_alt(b, sizeof(b), a->gnss_baro_diff_ft);
        det_row("GNSS - baro", "%s", b);
    }
    det_row("On ground", "%s", a->ground_valid ? (a->on_ground ? "Yes" : "No (airborne)") : "-");
    if (a->pos_valid && a->nic) det_row("NIC", "%u", a->nic);

    det_hdr("MOTION");
    if (a->gs_ms) { fmt_speed(b, sizeof(b), a->gs_kt); det_row("Ground speed", "%s", b); }
    else det_row("Ground speed", "-");
    if (a->track_ms) det_row("Track", "%.1f\xC2\xB0", a->track_deg);
    if (a->heading_ms) det_row("Heading", "%.1f\xC2\xB0 %s", a->heading_deg, a->heading_true && a->hrd_valid ? "true" : "magnetic");
    if (a->tas_ms) { fmt_speed(b, sizeof(b), a->tas_kt); det_row("True airspeed", "%s", b); }
    if (a->ias_ms) { fmt_speed(b, sizeof(b), a->ias_kt); det_row("Indicated airspeed", "%s", b); }
    if (a->mach_ms) det_row("Mach", "%.3f", a->mach);
    if (a->baro_rate_ms) { fmt_vrate(b, sizeof(b), a->baro_rate_fpm); det_row("Vertical rate (baro)", "%s", b); }
    if (a->geom_rate_ms) { fmt_vrate(b, sizeof(b), a->geom_rate_fpm); det_row("Vertical rate (GNSS/INS)", "%s", b); }
    if (a->roll_ms) det_row("Roll angle", "%.1f\xC2\xB0 %s", fabsf(a->roll_deg),
                            a->roll_deg > 0.5f ? "right" : a->roll_deg < -0.5f ? "left" : "level");
    if (a->track_rate_ms) det_row("Track rate", "%.2f\xC2\xB0/s", a->track_rate);

    det_hdr("AUTOPILOT / INTENT");
    bool any = false;
    if (a->sel_alt_ms) { fmt_alt(b, sizeof(b), a->sel_alt_ft); det_row(a->sel_alt_fms ? "Selected alt (FMS)" : "Selected alt (MCP/FCU)", "%s", b); any = true; }
    if (a->fms_alt_ms) { fmt_alt(b, sizeof(b), a->fms_alt_ft); det_row("FMS altitude", "%s", b); any = true; }
    if (a->qnh_ms) { det_row("Altimeter setting", "%.1f hPa  (%.2f inHg)", a->qnh_hpa, a->qnh_hpa * 0.02953f); any = true; }
    if (a->sel_hdg_ms) { det_row("Selected heading", "%.0f\xC2\xB0", a->sel_hdg_deg); any = true; }
    if (a->nav_modes_valid) {
        snprintf(b, sizeof(b), "%s%s%s%s%s%s",
                 a->ap_on ? "AP " : "", a->vnav_on ? "VNAV " : "", a->alt_hold_on ? "ALT-HOLD " : "",
                 a->approach_on ? "APPR " : "", a->lnav_on ? "LNAV " : "", a->tcas_on ? "TCAS " : "");
        det_row("Engaged modes", "%s", b[0] ? b : "none");
        any = true;
    }
    if (!any) det_row("Intent data", "not received (needs ADS-B v2 or Comm-B 4,0)");

    det_hdr("STATUS");
    det_row("Emergency", "%s", a->emergency_valid ? emergency_str(a->emergency) : "None reported");
    det_row("Alert / SPI", "%s / %s", a->alert ? "ALERT" : "no", a->spi ? "IDENT" : "no");
    det_row("ACAS / TCAS RA", "%s", a->acas_ra_ms && t - a->acas_ra_ms < 30000 && a->acas_ra_active ? "RESOLUTION ADVISORY ACTIVE" : "none");
    if (a->acas_op_valid) det_row("ACAS operational", "%s", a->acas_operational ? "Yes" : "No");
    if (a->cap_valid) det_row("Receives", "%s%s%s", a->es_in ? "1090ES-IN " : "", a->uat_in ? "UAT-IN" : "",
                              !a->es_in && !a->uat_in ? "none" : "");

    det_hdr("DERIVED");
    if (a->oat_ms && t - a->oat_ms < 60000) det_row("Outside air temp", "%.0f \xC2\xB0""C  (from TAS/Mach)", a->oat_c);
    else det_row("Outside air temp", "- (needs TAS + Mach)");
    if (a->wind_ms && t - a->wind_ms < 60000) {
        fmt_speed(b, sizeof(b), a->wind_kt);
        det_row("Wind", "%03.0f\xC2\xB0 / %s", a->wind_dir, b);
    }

    det_hdr("SIGNAL / QUALITY");
    if (!isnan(a->rssi)) det_row("Signal (RSSI)", "%.1f dBFS", a->rssi);
    det_row("Messages", "%lu  (%.1f/s)", (unsigned long)a->msgs, a->msg_rate);
    det_row("First / last seen", "%s / %s", age_str(t, a->first_seen_ms, b, sizeof(b)), age_str(t, a->last_seen_ms, c, sizeof(c)));
    if (a->nac_p_valid) det_row("NACp (position accuracy)", "%u  %s", a->nac_p, nacp_str(a->nac_p));
    if (a->nac_v_valid) det_row("NACv (velocity accuracy)", "%u  %s", a->nac_v, nacv_str(a->nac_v));
    if (a->sil_valid) det_row("SIL (integrity)", "%u  %s", a->sil, sil_str(a->sil));
    if (a->sda_valid) det_row("SDA (design assurance)", "%u", a->sda);
    if (a->nic_baro_valid) det_row("NIC baro", "%s", a->nic_baro ? "cross-checked" : "not cross-checked");
    if (a->gva_valid) det_row("GVA (vertical accuracy)", "%s", a->gva == 0 ? "unknown" : a->gva == 1 ? "<150 m" : "<45 m");
    if (a->bds_seen) {
        snprintf(b, sizeof(b), "%s%s%s%s%s%s",
                 a->bds_seen & 1 ? "1,0 " : "", a->bds_seen & 2 ? "2,0 " : "", a->bds_seen & 4 ? "3,0 " : "",
                 a->bds_seen & 8 ? "4,0 " : "", a->bds_seen & 16 ? "5,0 " : "", a->bds_seen & 32 ? "6,0 " : "");
        det_row("Comm-B registers", "%s", b);
    }
}

static void det_table_draw_cb(lv_event_t *e)
{
    lv_draw_task_t *t = lv_event_get_draw_task(e);
    lv_draw_dsc_base_t *base = lv_draw_task_get_draw_dsc(t);
    if (base->part != LV_PART_ITEMS || base->id1 >= (uint32_t)s_det_n) return;
    const det_row_t *r = &s_det_rows[base->id1];
    lv_draw_fill_dsc_t *f = lv_draw_task_get_fill_dsc(t);
    lv_draw_label_dsc_t *l = lv_draw_task_get_label_dsc(t);
    if (r->header) {
        if (f) f->color = lv_color_hex(0x1E293B);
        if (l) { l->color = COL_ACCENT; l->font = &lv_font_montserrat_14; }
    } else {
        if (f) f->color = COL_PANEL;
        if (l && base->id2 == 0) l->color = COL_DIM;
        if (l && base->id2 == 1 && (strstr(r->value, "EMERG") || strstr(r->value, "ACTIVE") ||
                                    strstr(r->value, "UNLAWFUL") || strstr(r->value, "Hijack"))) {
            l->color = COL_EMERG;
        }
    }
}

/*
 * LVGL's TJPGD decoder only recognises JPEGs that start with a JFIF APP0
 * header. Many photos start with an Exif APP1 segment instead, so replace any
 * leading APPn segments with a minimal JFIF header. Returns a new buffer
 * (old one freed) or the original if it was already JFIF.
 */
static uint8_t *jpeg_to_jfif(uint8_t *jpg, size_t *len)
{
    static const uint8_t jfif[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00,
                                   0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00};
    if (*len > 10 && memcmp(jpg, jfif, 10) == 0) return jpg;
    size_t p = 2;
    while (p + 4 <= *len && jpg[p] == 0xFF && jpg[p + 1] >= 0xE0 && jpg[p + 1] <= 0xEF) {
        p += 2 + ((jpg[p + 2] << 8) | jpg[p + 3]);   /* skip APPn segment */
    }
    if (p >= *len) return jpg;
    size_t nlen = sizeof(jfif) + (*len - p);
    uint8_t *out = heap_caps_malloc(nlen, MALLOC_CAP_SPIRAM);
    if (!out) return jpg;
    memcpy(out, jfif, sizeof(jfif));
    memcpy(out + sizeof(jfif), jpg + p, *len - p);
    free(jpg);
    *len = nlen;
    return out;
}

static void update_photo(void)
{
    uint32_t addr;
    size_t len;
    uint8_t *jpg = lookup_photo_take(&addr, &len);
    if (!jpg) return;
    if (addr != s_selected) { free(jpg); return; }
    jpg = jpeg_to_jfif(jpg, &len);

    int next = s_photo_idx ^ 1;
    lv_image_dsc_t *d = &s_photo_dsc[next];
    memset(d, 0, sizeof(*d));
    d->header.magic = LV_IMAGE_HEADER_MAGIC;
    d->header.cf = LV_COLOR_FORMAT_RAW;
    d->data = jpg;
    d->data_size = len;
    lv_image_set_src(s_det_photo, d);
    lv_obj_remove_flag(s_det_photo, LV_OBJ_FLAG_HIDDEN);

    /* release the previous image */
    if (s_photo_buf[s_photo_idx]) {
        lv_image_cache_drop(&s_photo_dsc[s_photo_idx]);
        free(s_photo_buf[s_photo_idx]);
        s_photo_buf[s_photo_idx] = NULL;
    }
    s_photo_buf[next] = jpg;
    s_photo_idx = next;
    s_photo_addr = addr;
}

static void update_details(void)
{
    if (!s_sel_valid) {
        lv_obj_remove_flag(s_det_empty, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_det_table, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_det_title, s_selected ? "Aircraft lost" : "No aircraft selected");
        lv_label_set_text(s_det_sub1, "");
        lv_label_set_text(s_det_sub2, "");
        lv_label_set_text(s_det_route, "");
        lv_obj_add_flag(s_det_photo, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    const aircraft_t *a = s_sel_copy;
    lv_obj_add_flag(s_det_empty, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_det_table, LV_OBJ_FLAG_HIDDEN);
    if (s_photo_addr != a->addr) lv_obj_add_flag(s_det_photo, LV_OBJ_FLAG_HIDDEN);

    char b[128];
    if (a->callsign[0]) lv_label_set_text(s_det_title, a->callsign);
    else lv_label_set_text_fmt(s_det_title, "%06lX", (unsigned long)(a->addr & 0xFFFFFF));
    snprintf(b, sizeof(b), "%s%s%s%s%s", a->registration[0] ? a->registration : "",
             a->registration[0] && a->type_code[0] ? "  \xE2\x80\xA2  " : "", a->type_code,
             a->military ? "  \xE2\x80\xA2  MIL" : "",
             a->emergency_valid && a->emergency ? "  \xE2\x80\xA2  EMERGENCY" : "");
    lv_label_set_text(s_det_sub1, b);
    snprintf(b, sizeof(b), "%s%s%s", a->type_name[0] ? a->type_name : (a->manufacturer[0] ? a->manufacturer : ""),
             (a->type_name[0] || a->manufacturer[0]) && (a->airline[0] || a->owner[0]) ? "\n" : "",
             a->airline[0] ? a->airline : a->owner);
    lv_label_set_text(s_det_sub2, b);
    if (a->origin.valid || a->destination.valid) {
        snprintf(b, sizeof(b), "%s  " LV_SYMBOL_RIGHT "  %s",
                 a->origin.valid ? (a->origin.iata[0] ? a->origin.iata : a->origin.icao) : "?",
                 a->destination.valid ? (a->destination.iata[0] ? a->destination.iata : a->destination.icao) : "?");
        lv_label_set_text(s_det_route, b);
    } else {
        lv_label_set_text(s_det_route, a->country ? a->country : "");
    }

    build_details_rows(a);
    lv_table_set_row_count(s_det_table, s_det_n);
    for (int i = 0; i < s_det_n; i++) {
        lv_table_set_cell_value(s_det_table, i, 0, s_det_rows[i].name ? s_det_rows[i].name : "");
        lv_table_set_cell_value(s_det_table, i, 1, s_det_rows[i].value);
    }
}

static void build_details_tab(lv_obj_t *tab)
{
    lv_obj_set_style_pad_all(tab, 8, 0);
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(tab, 6, 0);

    lv_obj_t *head = lv_obj_create(tab);
    lv_obj_remove_style_all(head);
    lv_obj_set_size(head, lv_pct(100), 150);
    lv_obj_remove_flag(head, LV_OBJ_FLAG_SCROLLABLE);

    s_det_photo = lv_image_create(head);
    lv_obj_set_size(s_det_photo, 220, 146);
    lv_obj_align(s_det_photo, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_image_set_inner_align(s_det_photo, LV_IMAGE_ALIGN_CONTAIN);
    lv_obj_add_flag(s_det_photo, LV_OBJ_FLAG_HIDDEN);

    s_det_title = lv_label_create(head);
    lv_obj_set_style_text_font(s_det_title, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(s_det_title, COL_SEL, 0);
    lv_obj_align(s_det_title, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_label_set_text(s_det_title, "No aircraft selected");

    s_det_sub1 = lv_label_create(head);
    lv_obj_set_style_text_font(s_det_sub1, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_det_sub1, COL_TEXT, 0);
    lv_obj_align(s_det_sub1, LV_ALIGN_TOP_LEFT, 0, 42);
    lv_label_set_text(s_det_sub1, "");

    s_det_sub2 = lv_label_create(head);
    lv_obj_set_style_text_color(s_det_sub2, COL_DIM, 0);
    lv_obj_set_width(s_det_sub2, 540);
    lv_label_set_long_mode(s_det_sub2, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_align(s_det_sub2, LV_ALIGN_TOP_LEFT, 0, 72);
    lv_label_set_text(s_det_sub2, "");

    s_det_route = lv_label_create(head);
    lv_obj_set_style_text_font(s_det_route, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_det_route, COL_ACCENT, 0);
    lv_obj_align(s_det_route, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_label_set_text(s_det_route, "");

    s_det_empty = lv_label_create(tab);
    lv_obj_set_style_text_color(s_det_empty, COL_DIM, 0);
    lv_label_set_text(s_det_empty, "Tap an aircraft on the radar or in the list.");

    s_det_table = lv_table_create(tab);
    lv_obj_set_width(s_det_table, lv_pct(100));
    lv_obj_set_height(s_det_table, LV_SIZE_CONTENT);
    lv_table_set_column_count(s_det_table, 2);
    lv_table_set_column_width(s_det_table, 0, 270);
    lv_table_set_column_width(s_det_table, 1, 500);
    lv_obj_set_style_pad_ver(s_det_table, 4, LV_PART_ITEMS);
    lv_obj_set_style_border_width(s_det_table, 0, LV_PART_ITEMS);
    lv_obj_set_style_text_font(s_det_table, &lv_font_montserrat_16, LV_PART_ITEMS);
    lv_obj_add_flag(s_det_table, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);
    lv_obj_add_event_cb(s_det_table, det_table_draw_cb, LV_EVENT_DRAW_TASK_ADDED, NULL);
    lv_obj_add_flag(s_det_table, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_det_table, LV_OBJ_FLAG_SCROLLABLE);
}

/* ------------------------------------------------------------------------- */
/* Stats                                                                     */
/* ------------------------------------------------------------------------- */

static void build_stats_tab(lv_obj_t *tab)
{
    lv_obj_set_style_pad_all(tab, 10, 0);
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);

    s_stats_chart = lv_chart_create(tab);
    lv_obj_set_size(s_stats_chart, lv_pct(100), 150);
    lv_chart_set_type(s_stats_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_stats_chart, RATE_HIST);
    lv_chart_set_update_mode(s_stats_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_obj_set_style_size(s_stats_chart, 0, 0, LV_PART_INDICATOR);
    lv_chart_set_div_line_count(s_stats_chart, 4, 0);
    s_rate_ser = lv_chart_add_series(s_stats_chart, COL_ACCENT, LV_CHART_AXIS_PRIMARY_Y);

    lv_obj_t *row = lv_obj_create(tab);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);

    s_stats_lbl = lv_label_create(row);
    lv_obj_set_width(s_stats_lbl, 480);
    lv_obj_set_style_text_color(s_stats_lbl, COL_TEXT, 0);

    s_df_lbl = lv_label_create(row);
    lv_obj_set_width(s_df_lbl, 280);
    lv_obj_set_style_text_color(s_df_lbl, COL_DIM, 0);
    lv_obj_set_style_text_font(s_df_lbl, &lv_font_montserrat_14, 0);
}

static void update_stats(void)
{
    feed_stats_t st;
    feed_status_t fs;
    lookup_stats_t ls;
    aircraft_lock();
    st = *aircraft_stats();
    aircraft_unlock();
    adsb_client_status(&fs);
    lookup_get_stats(&ls);

    lv_chart_set_next_value(s_stats_chart, s_rate_ser, (int32_t)st.msg_rate);
    int32_t maxv = 10;
    int32_t *ys = lv_chart_get_series_y_array(s_stats_chart, s_rate_ser);
    for (int i = 0; i < RATE_HIST; i++) if (ys[i] != LV_CHART_POINT_NONE && ys[i] > maxv) maxv = ys[i];
    lv_chart_set_axis_range(s_stats_chart, LV_CHART_AXIS_PRIMARY_Y, 0, maxv + maxv / 5);

    char ip[20];
    wifi_ip_str(ip, sizeof(ip));
    uint32_t up = now_ms() / 1000;
    const app_settings_t *cfg = settings_get();
    char rng[24];
    fmt_dist(rng, sizeof(rng), st.max_range_nm);
    lv_label_set_text_fmt(s_stats_lbl,
        "Messages/s: %.0f   (peak chart above)\n"
        "Messages: %lu total, %lu decoded, %lu rejected\n"
        "Aircraft: %d tracked, %d with position, %lu seen since boot\n"
        "Max range seen: %s\n"
        "Feed: %s  %s\n"
        "Bytes received: %lu, reconnects: %lu\n"
        "Wi-Fi: %s, %d dBm, IP %s\n"
        "Lookups: %lu req, %lu ok, %lu unknown, %lu errors\n"
        "Uptime: %luh %02lum   Heap: %u KB int / %u KB PSRAM\n"
        "Firmware: %s",
        st.msg_rate, (unsigned long)st.msgs_total, (unsigned long)st.msgs_good, (unsigned long)st.msgs_bad,
        s_snap_n, s_with_pos, (unsigned long)st.aircraft_total,
        cfg->rx_pos_valid ? rng : "(set receiver location)",
        feed_state_str(fs.state), fs.detail,
        (unsigned long)fs.bytes_rx, (unsigned long)(fs.connects ? fs.connects - 1 : 0),
        wifi_state_str(), wifi_rssi(), ip,
        (unsigned long)ls.requests, (unsigned long)ls.ok, (unsigned long)ls.not_found, (unsigned long)ls.errors,
        (unsigned long)(up / 3600), (unsigned long)((up / 60) % 60),
        (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
        (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
        esp_app_get_description()->version);

    lv_label_set_text_fmt(s_df_lbl,
        "Downlink formats\n"
        "DF0  short ACAS   %lu\n"
        "DF4  altitude     %lu\n"
        "DF5  identity     %lu\n"
        "DF11 all-call     %lu\n"
        "DF16 long ACAS    %lu\n"
        "DF17 ADS-B        %lu\n"
        "DF18 TIS-B/ADS-R  %lu\n"
        "DF20 Comm-B alt   %lu\n"
        "DF21 Comm-B id    %lu",
        (unsigned long)st.df_count[0], (unsigned long)st.df_count[4], (unsigned long)st.df_count[5],
        (unsigned long)st.df_count[11], (unsigned long)st.df_count[16], (unsigned long)st.df_count[17],
        (unsigned long)st.df_count[18], (unsigned long)st.df_count[20], (unsigned long)st.df_count[21]);
}

/* ------------------------------------------------------------------------- */
/* Settings                                                                  */
/* ------------------------------------------------------------------------- */

static void kb_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_obj_add_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_t *ta = lv_keyboard_get_textarea(s_kb);
        if (ta) lv_obj_remove_state(ta, LV_STATE_FOCUSED);
    }
}

static void ta_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target(e);
    if (code == LV_EVENT_FOCUSED || code == LV_EVENT_CLICKED) {
        bool numeric = (bool)(intptr_t)lv_event_get_user_data(e);
        lv_keyboard_set_mode(s_kb, numeric ? LV_KEYBOARD_MODE_NUMBER : LV_KEYBOARD_MODE_TEXT_LOWER);
        lv_keyboard_set_textarea(s_kb, ta);
        lv_obj_remove_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
    } else if (code == LV_EVENT_DEFOCUSED) {
        lv_obj_add_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static lv_obj_t *form_row(lv_obj_t *parent, const char *label)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 10, 0);
    lv_obj_set_style_pad_ver(row, 4, 0);
    lv_obj_t *l = lv_label_create(row);
    lv_label_set_text(l, label);
    lv_obj_set_width(l, 190);
    lv_obj_set_style_text_color(l, COL_DIM, 0);
    return row;
}

static lv_obj_t *form_ta(lv_obj_t *row, const char *txt, int32_t w, bool numeric, bool password)
{
    lv_obj_t *ta = lv_textarea_create(row);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_text(ta, txt);
    lv_textarea_set_password_mode(ta, password);
    lv_obj_set_width(ta, w);
    lv_obj_add_event_cb(ta, ta_event_cb, LV_EVENT_ALL, (void *)(intptr_t)numeric);
    return ta;
}

static void scan_cb(lv_event_t *e)
{
    wifi_scan_start();
    s_scan_pending = true;
    lv_label_set_text(lv_obj_get_child(s_btn_scan, 0), "Scanning...");
}

static void scan_pick_cb(lv_event_t *e)
{
    char buf[40];
    lv_dropdown_get_selected_str(s_dd_scan, buf, sizeof(buf));
    char *sp = strstr(buf, "  (");
    if (sp) *sp = 0;
    if (buf[0] && strcmp(buf, "Networks") != 0) lv_textarea_set_text(s_ta_ssid, buf);
}

static void poll_scan(void)
{
    if (!s_scan_pending) return;
    wifi_ap_info_t aps[WIFI_SCAN_MAX];
    int n = wifi_scan_results(aps, WIFI_SCAN_MAX);
    if (n < 0) return;
    s_scan_pending = false;
    lv_label_set_text(lv_obj_get_child(s_btn_scan, 0), LV_SYMBOL_REFRESH " Scan");
    static char opts[WIFI_SCAN_MAX * 48];
    strcpy(opts, "Networks");
    for (int i = 0; i < n; i++) {
        char line[80];
        snprintf(line, sizeof(line), "\n%.32s  (%d dBm%s)", aps[i].ssid, aps[i].rssi, aps[i].secure ? "" : ", open");
        strlcat(opts, line, sizeof(opts));
    }
    lv_dropdown_set_options(s_dd_scan, opts);
    if (n) lv_dropdown_open(s_dd_scan);
}

static void proto_changed_cb(lv_event_t *e)
{
    static const char *ports[] = {"30005", "30002", "30003"};
    const char *cur = lv_textarea_get_text(s_ta_port);
    bool is_default = !cur[0] || !strcmp(cur, ports[0]) || !strcmp(cur, ports[1]) || !strcmp(cur, ports[2]);
    if (is_default) lv_textarea_set_text(s_ta_port, ports[lv_dropdown_get_selected(s_dd_proto)]);
}

static void vcom_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(s_sl_vcom);
    lv_label_set_text_fmt(s_lbl_vcom, "0x%02X", v);
    /* apply on release only: each change is a few DSI commands */
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) {
        settings_get()->vcom = (uint8_t)v;
        board_set_vcom((uint8_t)v);
    }
}

static void bright_cb(lv_event_t *e)
{
    int v = lv_slider_get_value(s_sl_bright);
    settings_get()->brightness = (uint8_t)v;
    board_set_backlight((uint8_t)v);
}

static const uint16_t s_saver_steps[] = {1, 5, 10, 15, 30, 60, 120};
#define SAVER_STEPS (int)(sizeof(s_saver_steps) / sizeof(s_saver_steps[0]))

static int saver_index(uint16_t minutes)
{
    for (int i = 0; i < SAVER_STEPS; i++) if (s_saver_steps[i] >= minutes) return i;
    return SAVER_STEPS - 1;
}

static void saver_cb(lv_event_t *e)
{
    app_settings_t *c = settings_get();
    c->saver = lv_obj_has_state(s_sw_saver, LV_STATE_CHECKED);
    c->saver_min = s_saver_steps[lv_dropdown_get_selected(s_dd_saver)];
    lv_display_trigger_activity(NULL);   /* restart the countdown from now */
}

/* Called once a second: blank the backlight after the inactivity timeout */
static void screen_saver_tick(void)
{
    const app_settings_t *c = settings_get();
    if (!c->saver || board_screen_is_off()) return;
    if (lv_display_get_inactive_time(NULL) >= (uint32_t)c->saver_min * 60000u) {
        lv_obj_add_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
        board_screen_off();
    }
}

static void flip_cb(lv_event_t *e)
{
    bool f = lv_dropdown_get_selected(s_dd_flip) == 1;
    settings_get()->flip = f;
    board_set_flip(f);
}

/* ---- firmware section ---- */

typedef void (*confirm_fn_t)(void);

static void confirm_btn_cb(lv_event_t *e)
{
    confirm_fn_t fn = (confirm_fn_t)lv_event_get_user_data(e);
    lv_obj_t *mbox = lv_obj_get_parent(lv_obj_get_parent(lv_event_get_target(e)));
    lv_msgbox_close(mbox);
    if (fn) fn();
}

static void confirm(const char *title, const char *text, const char *ok_label, confirm_fn_t fn)
{
    lv_obj_t *mb = lv_msgbox_create(NULL);
    lv_obj_set_width(mb, 640);
    lv_msgbox_add_title(mb, title);
    lv_msgbox_add_text(mb, text);
    lv_obj_t *ok = lv_msgbox_add_footer_button(mb, ok_label);
    lv_obj_add_event_cb(ok, confirm_btn_cb, LV_EVENT_CLICKED, (void *)fn);
    lv_obj_t *cancel = lv_msgbox_add_footer_button(mb, "Cancel");
    lv_obj_add_event_cb(cancel, confirm_btn_cb, LV_EVENT_CLICKED, NULL);
}

static void do_revert(void)
{
    if (!ota_revert()) lv_label_set_text(s_fw_status, LV_SYMBOL_WARNING " No previous firmware to revert to");
}

static void do_c6_update(void)
{
    if (!ota_c6_update_start()) lv_label_set_text(s_c6_lbl, LV_SYMBOL_WARNING " C6 update could not start (busy?)");
}

static void arm_cb(lv_event_t *e) { ota_arm(); }

static void revert_cb(lv_event_t *e)
{
    ota_status_t st;
    ota_get_status(&st);
    char msg[160];
    snprintf(msg, sizeof(msg), "Switch from %s back to %s and reboot?", st.version, st.prev_version);
    confirm("Revert firmware", msg, "Revert", do_revert);
}

static void c6_cb(lv_event_t *e)
{
    confirm("Update Wi-Fi co-processor",
            "Send ESP-Hosted firmware to the ESP32-C6 over SDIO, then restart.\n"
            "Takes about a minute. Keep the display powered.\n"
            "If the transfer fails the C6 keeps its current firmware.",
            "Update C6", do_c6_update);
}

static void update_fw_section(void)
{
    ota_status_t st;
    ota_get_status(&st);
    char ip[20];
    wifi_ip_str(ip, sizeof(ip));

    lv_label_set_text_fmt(s_fw_lbl, "Firmware %s (%s, %s)%s\nPrevious: %s",
                          st.version, st.partition, st.build_date,
                          st.pending_verify ? "  - verifying..." : "",
                          st.can_revert ? st.prev_version : "none");
    if (st.can_revert) lv_obj_remove_state(s_btn_revert, LV_STATE_DISABLED);
    else lv_obj_add_state(s_btn_revert, LV_STATE_DISABLED);

    if (st.state == OTA_RECEIVING && st.total) {
        lv_obj_remove_flag(s_fw_bar, LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(s_fw_bar, (int32_t)((uint64_t)st.received * 100 / st.total), LV_ANIM_OFF);
        lv_label_set_text_fmt(s_fw_status, "Receiving update: %lu / %lu KB",
                              (unsigned long)(st.received / 1024), (unsigned long)(st.total / 1024));
    } else if (st.armed_left_s) {
        lv_obj_add_flag(s_fw_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text_fmt(s_fw_status, "Upload at http://%s/  (%lu:%02lu left)", ip,
                              (unsigned long)(st.armed_left_s / 60), (unsigned long)(st.armed_left_s % 60));
    } else {
        lv_obj_add_flag(s_fw_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_fw_status, st.message[0] ? st.message : "Uploads closed");
    }

    if (st.c6_busy && st.c6_total) {
        lv_label_set_text_fmt(s_c6_lbl, "Wi-Fi chip: updating %lu%%", (unsigned long)(st.c6_sent * 100 / st.c6_total));
    } else if (st.c6_message[0]) {
        lv_label_set_text(s_c6_lbl, st.c6_message);
    } else {
        lv_label_set_text_fmt(s_c6_lbl, "Wi-Fi chip (C6): %s, bundled %s%s",
                              st.c6_version_known ? st.c6_version : "?", st.c6_bundled,
                              st.c6_update_available ? "  - update available" : "");
    }
    if (st.c6_update_available && !st.c6_busy && !st.pending_verify) lv_obj_remove_state(s_btn_c6, LV_STATE_DISABLED);
    else lv_obj_add_state(s_btn_c6, LV_STATE_DISABLED);
}

static void save_cb(lv_event_t *e)
{
    app_settings_t *c = settings_get();
    char old_ssid[33], old_pass[65], old_host[64];
    uint16_t old_port = c->port;
    feed_proto_t old_proto = c->proto;
    strlcpy(old_ssid, c->wifi_ssid, sizeof(old_ssid));
    strlcpy(old_pass, c->wifi_pass, sizeof(old_pass));
    strlcpy(old_host, c->host, sizeof(old_host));

    strlcpy(c->wifi_ssid, lv_textarea_get_text(s_ta_ssid), sizeof(c->wifi_ssid));
    strlcpy(c->wifi_pass, lv_textarea_get_text(s_ta_pass), sizeof(c->wifi_pass));
    strlcpy(c->host, lv_textarea_get_text(s_ta_host), sizeof(c->host));
    int port = atoi(lv_textarea_get_text(s_ta_port));
    c->port = (port > 0 && port < 65536) ? (uint16_t)port : 30005;
    c->proto = (feed_proto_t)lv_dropdown_get_selected(s_dd_proto);
    c->rx_lat = strtod(lv_textarea_get_text(s_ta_lat), NULL);
    c->rx_lon = strtod(lv_textarea_get_text(s_ta_lon), NULL);
    if (c->rx_lat < -90 || c->rx_lat > 90) c->rx_lat = 0;
    if (c->rx_lon < -180 || c->rx_lon > 180) c->rx_lon = 0;
    c->units = (units_t)lv_dropdown_get_selected(s_dd_units);
    c->tz = (uint8_t)lv_dropdown_get_selected(s_dd_tz);
    c->flip = lv_dropdown_get_selected(s_dd_flip) == 1;
    c->saver = lv_obj_has_state(s_sw_saver, LV_STATE_CHECKED);
    c->saver_min = s_saver_steps[lv_dropdown_get_selected(s_dd_saver)];
    settings_apply_tz();
    c->online_lookup = lv_obj_has_state(s_sw_lookup, LV_STATE_CHECKED);
    c->vcom = (uint8_t)lv_slider_get_value(s_sl_vcom);
    settings_save();

    if (strcmp(old_ssid, c->wifi_ssid) || strcmp(old_pass, c->wifi_pass)) wifi_apply_settings();
    if (strcmp(old_host, c->host) || old_port != c->port || old_proto != c->proto) adsb_client_restart();

    s_center_on_rx = true;
    s_center_valid = false;
    update_range_label();
    lv_obj_add_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_set_status, LV_SYMBOL_OK " Saved");
    lv_obj_set_style_text_color(s_set_status, COL_OK, 0);
}

static void build_settings_tab(lv_obj_t *tab)
{
    const app_settings_t *c = settings_get();
    char b[32];
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(tab, 10, 0);
    lv_obj_set_style_pad_row(tab, 2, 0);

    lv_obj_t *r = form_row(tab, "Wi-Fi network");
    s_ta_ssid = form_ta(r, c->wifi_ssid, 300, false, false);
    s_btn_scan = lv_button_create(r);
    lv_obj_t *l = lv_label_create(s_btn_scan);
    lv_label_set_text(l, LV_SYMBOL_REFRESH " Scan");
    lv_obj_add_event_cb(s_btn_scan, scan_cb, LV_EVENT_CLICKED, NULL);

    r = form_row(tab, "");
    s_dd_scan = lv_dropdown_create(r);
    lv_dropdown_set_options(s_dd_scan, "Networks");
    lv_obj_set_width(s_dd_scan, 440);
    lv_obj_add_event_cb(s_dd_scan, scan_pick_cb, LV_EVENT_VALUE_CHANGED, NULL);

    r = form_row(tab, "Wi-Fi password");
    s_ta_pass = form_ta(r, c->wifi_pass, 440, false, true);

    r = form_row(tab, "ADS-B server");
    s_ta_host = form_ta(r, c->host, 300, false, false);
    snprintf(b, sizeof(b), "%u", c->port);
    s_ta_port = form_ta(r, b, 130, true, false);

    r = form_row(tab, "Feed protocol");
    s_dd_proto = lv_dropdown_create(r);
    lv_dropdown_set_options(s_dd_proto, "Beast binary (30005)\nRaw AVR (30002)\nSBS / BaseStation (30003)");
    lv_dropdown_set_selected(s_dd_proto, c->proto);
    lv_obj_set_width(s_dd_proto, 440);
    lv_obj_add_event_cb(s_dd_proto, proto_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    r = form_row(tab, "Receiver lat / lon");
    snprintf(b, sizeof(b), "%.5f", c->rx_lat);
    s_ta_lat = form_ta(r, b, 215, true, false);
    snprintf(b, sizeof(b), "%.5f", c->rx_lon);
    s_ta_lon = form_ta(r, b, 215, true, false);

    r = form_row(tab, "Units");
    s_dd_units = lv_dropdown_create(r);
    lv_dropdown_set_options(s_dd_units, "Aviation (ft, kt, nm)\nMetric (m, km/h, km)");
    lv_dropdown_set_selected(s_dd_units, c->units);
    lv_obj_set_width(s_dd_units, 440);

    r = form_row(tab, "Timezone");
    s_dd_tz = lv_dropdown_create(r);
    lv_dropdown_set_options(s_dd_tz, tz_dropdown_options());
    lv_dropdown_set_selected(s_dd_tz, c->tz);
    lv_obj_set_width(s_dd_tz, 440);

    r = form_row(tab, "Screen orientation");
    s_dd_flip = lv_dropdown_create(r);
    lv_dropdown_set_options(s_dd_flip, "Normal\nFlipped 180\xC2\xB0");
    lv_dropdown_set_selected(s_dd_flip, c->flip ? 1 : 0);
    lv_obj_set_width(s_dd_flip, 440);
    lv_obj_add_event_cb(s_dd_flip, flip_cb, LV_EVENT_VALUE_CHANGED, NULL);

    r = form_row(tab, "Online lookup");
    s_sw_lookup = lv_switch_create(r);
    if (c->online_lookup) lv_obj_add_state(s_sw_lookup, LV_STATE_CHECKED);
    l = lv_label_create(r);
    lv_label_set_text(l, "reg / type / route / photo via adsbdb.com");
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, COL_DIM, 0);

    r = form_row(tab, "Brightness");
    s_sl_bright = lv_slider_create(r);
    lv_slider_set_range(s_sl_bright, 5, 100);
    lv_slider_set_value(s_sl_bright, c->brightness, LV_ANIM_OFF);
    lv_obj_set_width(s_sl_bright, 420);
    lv_obj_add_event_cb(s_sl_bright, bright_cb, LV_EVENT_VALUE_CHANGED, NULL);

    r = form_row(tab, "Screen saver");
    s_sw_saver = lv_switch_create(r);
    if (c->saver) lv_obj_add_state(s_sw_saver, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_sw_saver, saver_cb, LV_EVENT_VALUE_CHANGED, NULL);
    s_dd_saver = lv_dropdown_create(r);
    lv_dropdown_set_options(s_dd_saver, "after 1 min\nafter 5 min\nafter 10 min\nafter 15 min\n"
                                        "after 30 min\nafter 1 hour\nafter 2 hours");
    lv_dropdown_set_selected(s_dd_saver, saver_index(c->saver_min));
    lv_obj_set_width(s_dd_saver, 300);
    lv_obj_add_event_cb(s_dd_saver, saver_cb, LV_EVENT_VALUE_CHANGED, NULL);

    r = form_row(tab, "Panel VCOM");
    s_sl_vcom = lv_slider_create(r);
    lv_slider_set_range(s_sl_vcom, BOARD_VCOM_MIN, BOARD_VCOM_MAX);
    lv_slider_set_value(s_sl_vcom, c->vcom, LV_ANIM_OFF);
    lv_obj_set_width(s_sl_vcom, 330);
    lv_obj_add_event_cb(s_sl_vcom, vcom_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_sl_vcom, vcom_cb, LV_EVENT_RELEASED, NULL);
    s_lbl_vcom = lv_label_create(r);
    lv_label_set_text_fmt(s_lbl_vcom, "0x%02X", c->vcom);
    l = lv_label_create(r);
    lv_label_set_text(l, "adjust if old screens ghost");
    lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(l, COL_DIM, 0);

    r = form_row(tab, "");
    lv_obj_t *save = lv_button_create(r);
    lv_obj_set_size(save, 260, 56);
    lv_obj_set_style_bg_color(save, COL_OK, 0);
    l = lv_label_create(save);
    lv_label_set_text(l, LV_SYMBOL_SAVE "  Save & apply");
    lv_obj_center(l);
    lv_obj_add_event_cb(save, save_cb, LV_EVENT_CLICKED, NULL);
    s_set_status = lv_label_create(r);
    lv_label_set_text(s_set_status, "");

    /* ---- firmware / OTA ---- */
    l = lv_label_create(tab);
    lv_label_set_text(l, "FIRMWARE");
    lv_obj_set_style_text_color(l, COL_ACCENT, 0);
    lv_obj_set_style_pad_top(l, 14, 0);

    s_fw_lbl = lv_label_create(tab);
    lv_obj_set_width(s_fw_lbl, lv_pct(100));
    lv_obj_set_style_text_font(s_fw_lbl, &lv_font_montserrat_14, 0);

    r = form_row(tab, "");
    lv_obj_set_style_pad_left(lv_obj_get_child(r, 0), 0, 0);
    lv_obj_set_width(lv_obj_get_child(r, 0), 0);
    lv_obj_t *arm = lv_button_create(r);
    l = lv_label_create(arm);
    lv_label_set_text(l, LV_SYMBOL_UPLOAD "  Allow OTA update");
    lv_obj_add_event_cb(arm, arm_cb, LV_EVENT_CLICKED, NULL);
    s_btn_revert = lv_button_create(r);
    l = lv_label_create(s_btn_revert);
    lv_label_set_text(l, LV_SYMBOL_LOOP "  Revert to previous");
    lv_obj_add_event_cb(s_btn_revert, revert_cb, LV_EVENT_CLICKED, NULL);

    s_fw_status = lv_label_create(tab);
    lv_obj_set_width(s_fw_status, lv_pct(100));
    lv_obj_set_style_text_color(s_fw_status, COL_DIM, 0);
    lv_obj_set_style_text_font(s_fw_status, &lv_font_montserrat_14, 0);
    s_fw_bar = lv_bar_create(tab);
    lv_obj_set_size(s_fw_bar, lv_pct(95), 12);
    lv_obj_add_flag(s_fw_bar, LV_OBJ_FLAG_HIDDEN);

    r = form_row(tab, "");
    lv_obj_set_width(lv_obj_get_child(r, 0), 0);
    s_btn_c6 = lv_button_create(r);
    l = lv_label_create(s_btn_c6);
    lv_label_set_text(l, LV_SYMBOL_WIFI "  Update Wi-Fi chip");
    lv_obj_add_event_cb(s_btn_c6, c6_cb, LV_EVENT_CLICKED, NULL);
    s_c6_lbl = lv_label_create(r);
    lv_obj_set_width(s_c6_lbl, 480);
    lv_label_set_long_mode(s_c6_lbl, LV_LABEL_LONG_MODE_WRAP);
    lv_obj_set_style_text_font(s_c6_lbl, &lv_font_montserrat_14, 0);
    update_fw_section();

    /* keyboard lives on the top layer over the radar so the form stays visible */
    s_kb = lv_keyboard_create(lv_layer_top());
    lv_obj_set_size(s_kb, SCR_W, 420);
    lv_obj_align(s_kb, LV_ALIGN_TOP_MID, 0, TOPBAR_H);
    lv_obj_add_flag(s_kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_kb, kb_event_cb, LV_EVENT_ALL, NULL);
}

/* ------------------------------------------------------------------------- */
/* Periodic refresh                                                          */
/* ------------------------------------------------------------------------- */

static void select_aircraft(uint32_t addr, bool show_details)
{
    s_selected = addr;
    lookup_set_priority(addr);
    take_snapshot();
    update_details();
    update_list();
    lv_obj_invalidate(s_radar);
    if (show_details) lv_tabview_set_active(s_tabview, 1, LV_ANIM_OFF);
}

static void update_topbar(void)
{
    feed_status_t fs;
    adsb_client_status(&fs);
    feed_stats_t *st = aircraft_stats();

    if (wifi_is_connected()) lv_label_set_text_fmt(s_top_wifi, LV_SYMBOL_WIFI " %d dBm", wifi_rssi());
    else lv_label_set_text_fmt(s_top_wifi, LV_SYMBOL_WIFI " %s", wifi_state_str());
    lv_obj_set_style_text_color(s_top_wifi, wifi_is_connected() ? COL_OK : COL_WARN, 0);

    if (fs.state == FEED_CONNECTED) {
        lv_label_set_text_fmt(s_top_feed, LV_SYMBOL_DOWNLOAD " %s  %.0f msg/s", fs.detail, st->msg_rate);
        lv_obj_set_style_text_color(s_top_feed, COL_DIM, 0);
    } else {
        lv_label_set_text_fmt(s_top_feed, LV_SYMBOL_WARNING " %s %s", feed_state_str(fs.state), fs.detail);
        lv_obj_set_style_text_color(s_top_feed, COL_WARN, 0);
    }
    lv_label_set_text_fmt(s_top_count, LV_SYMBOL_GPS " %d / %d", s_with_pos, s_snap_n);

    if (time_is_synced()) {
        time_t now = time(NULL);
        struct tm lt, ut;
        localtime_r(&now, &lt);
        gmtime_r(&now, &ut);
        char zone[8];
        strftime(zone, sizeof(zone), "%Z", &lt);
        if (settings_get()->tz == 0) {
            lv_label_set_text_fmt(s_top_clock, "%02d:%02d:%02dZ", ut.tm_hour, ut.tm_min, ut.tm_sec);
            lv_label_set_text(s_top_utc, "");
        } else {
            lv_label_set_text_fmt(s_top_clock, "%02d:%02d:%02d %s", lt.tm_hour, lt.tm_min, lt.tm_sec, zone);
            lv_label_set_text_fmt(s_top_utc, "%02d:%02dZ  %02d/%02d", ut.tm_hour, ut.tm_min, lt.tm_mon + 1, lt.tm_mday);
        }
    } else {
        lv_label_set_text(s_top_clock, "--:--:--");
        lv_label_set_text(s_top_utc, "time not synced");
    }

    /* centre info */
    if (s_center_valid) {
        lv_label_set_text_fmt(s_center_lbl, "%s %.3f, %.3f", settings_get()->rx_pos_valid && s_center_on_rx ? "RX" : "Map",
                              s_clat, s_clon);
    } else {
        lv_label_set_text(s_center_lbl, "");
    }
}

static void update_banner(void)
{
    const snap_t *em = NULL;
    for (int i = 0; i < s_snap_n; i++) if (s_snap[i].emergency) { em = &s_snap[i]; break; }
    if (!em) {
        lv_obj_add_flag(s_banner, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    const char *why = em->sq_v && is_emergency_squawk(em->squawk) ? squawk_meaning(em->squawk) : "Emergency";
    char sq[8] = "";
    if (em->sq_v) snprintf(sq, sizeof(sq), "%04X", em->squawk);
    lv_label_set_text_fmt(s_banner_lbl, LV_SYMBOL_WARNING "  %s  %s  %s", em->callsign[0] ? em->callsign : "?", sq, why);
    lv_obj_set_user_data(s_banner, (void *)(uintptr_t)em->addr);
    lv_obj_remove_flag(s_banner, LV_OBJ_FLAG_HIDDEN);
    if (board_screen_is_off()) board_screen_wake();   /* never hide an emergency */
}

static void refresh_timer_cb(lv_timer_t *tm)
{
    aircraft_tick();
    take_snapshot();
    update_topbar();
    update_banner();
    screen_saver_tick();
    lv_obj_invalidate(s_radar);

    uint32_t active = lv_tabview_get_tab_active(s_tabview);
    if (active == 0) update_list();
    if (active == 3) update_fw_section();
    if (active == 1) update_details();
    update_stats();   /* also feeds the rate chart history */
    update_photo();
    poll_scan();
}

/* ------------------------------------------------------------------------- */

void ui_init(lv_display_t *disp)
{
    s_snap = heap_caps_calloc(AC_MAX, sizeof(snap_t), MALLOC_CAP_SPIRAM);
    s_sel_copy = heap_caps_calloc(1, sizeof(aircraft_t), MALLOC_CAP_SPIRAM);
    assert(s_snap && s_sel_copy);

    lv_theme_t *th = lv_theme_default_init(disp, COL_ACCENT, COL_SEL, true, &lv_font_montserrat_16);
    lv_display_set_theme(disp, th);

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    build_topbar(scr);
    build_radar(scr);

    s_tabview = lv_tabview_create(scr);
    lv_obj_set_pos(s_tabview, 0, PANEL_Y);
    lv_obj_set_size(s_tabview, SCR_W, PANEL_H);
    lv_tabview_set_tab_bar_size(s_tabview, 52);
    lv_obj_set_style_bg_color(s_tabview, COL_PANEL, 0);
    lv_obj_t *t_list = lv_tabview_add_tab(s_tabview, LV_SYMBOL_LIST "  Aircraft");
    lv_obj_t *t_det = lv_tabview_add_tab(s_tabview, LV_SYMBOL_EYE_OPEN "  Details");
    lv_obj_t *t_stats = lv_tabview_add_tab(s_tabview, LV_SYMBOL_BARS "  Stats");
    lv_obj_t *t_set = lv_tabview_add_tab(s_tabview, LV_SYMBOL_SETTINGS "  Settings");
    /* swiping between tabs fights with table scrolling; tabs are tapped instead */
    lv_obj_remove_flag(lv_tabview_get_content(s_tabview), LV_OBJ_FLAG_SCROLLABLE);

    build_list_tab(t_list);
    build_details_tab(t_det);
    build_stats_tab(t_stats);
    build_settings_tab(t_set);
    build_banner(scr);

    board_set_backlight(settings_get()->brightness);
    lv_timer_create(refresh_timer_cb, 1000, NULL);
    ota_report_ui_ready();
    ESP_LOGI(TAG, "UI ready");
}
