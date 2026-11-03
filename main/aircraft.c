#include "aircraft.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "icao_db.h"
#include "settings.h"

static const char *TAG = "aircraft";

static aircraft_t *s_ac;          /* AC_MAX entries in PSRAM */
static SemaphoreHandle_t s_lock;
static feed_stats_t s_stats;
static uint32_t s_rate_t0, s_rate_msgs;

#define CPR_PAIR_MAX_MS_AIR   10000
#define CPR_PAIR_MAX_MS_SURF  25000
#define FIELD_FRESH_MS        10000
#define MAX_RANGE_NM          450.0

uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void aircraft_init(void)
{
    s_ac = heap_caps_calloc(AC_MAX, sizeof(aircraft_t), MALLOC_CAP_SPIRAM);
    assert(s_ac);
    s_lock = xSemaphoreCreateMutex();
    s_rate_t0 = now_ms();
    ESP_LOGI(TAG, "aircraft table: %d x %u bytes", AC_MAX, (unsigned)sizeof(aircraft_t));
}

void aircraft_lock(void)   { xSemaphoreTake(s_lock, portMAX_DELAY); }
void aircraft_unlock(void) { xSemaphoreGive(s_lock); }

aircraft_t *aircraft_table(int *count)
{
    *count = AC_MAX;
    return s_ac;
}

feed_stats_t *aircraft_stats(void)
{
    return &s_stats;
}

aircraft_t *aircraft_find(uint32_t addr)
{
    for (int i = 0; i < AC_MAX; i++) {
        if (s_ac[i].used && s_ac[i].addr == addr) return &s_ac[i];
    }
    return NULL;
}

static bool known_addr_cb(uint32_t addr)
{
    return aircraft_find(addr) != NULL;
}

static aircraft_t *aircraft_create(uint32_t addr)
{
    aircraft_t *slot = NULL;
    uint32_t oldest = UINT32_MAX;
    for (int i = 0; i < AC_MAX; i++) {
        if (!s_ac[i].used) { slot = &s_ac[i]; break; }
        if (s_ac[i].last_seen_ms < oldest) { oldest = s_ac[i].last_seen_ms; slot = &s_ac[i]; }
    }
    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    slot->addr = addr;
    slot->first_seen_ms = slot->last_seen_ms = now_ms();
    slot->rate_window_start_ms = slot->first_seen_ms;
    slot->rssi = NAN;
    icao_lookup(addr, &slot->country, &slot->military);
    s_stats.aircraft_total++;
    return slot;
}

/* ------------------------------------------------------------------------- */
/* Geometry                                                                  */
/* ------------------------------------------------------------------------- */

static double deg2rad(double d) { return d * M_PI / 180.0; }

static double dist_nm(double lat1, double lon1, double lat2, double lon2)
{
    double p1 = deg2rad(lat1), p2 = deg2rad(lat2);
    double dp = p2 - p1, dl = deg2rad(lon2 - lon1);
    double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return 2.0 * 3440.065 * atan2(sqrt(a), sqrt(1 - a));
}

static float bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    double p1 = deg2rad(lat1), p2 = deg2rad(lat2), dl = deg2rad(lon2 - lon1);
    double y = sin(dl) * cos(p2);
    double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) * 180.0 / M_PI;
    return (float)(b < 0 ? b + 360.0 : b);
}

static void update_distance(aircraft_t *a)
{
    const app_settings_t *cfg = settings_get();
    if (!a->pos_valid || !cfg->rx_pos_valid) { a->dist_valid = false; return; }
    a->dist_nm = dist_nm(cfg->rx_lat, cfg->rx_lon, a->lat, a->lon);
    a->bearing_deg = bearing_deg(cfg->rx_lat, cfg->rx_lon, a->lat, a->lon);
    a->dist_valid = true;
    if (a->dist_nm > s_stats.max_range_nm && a->dist_nm < MAX_RANGE_NM) {
        s_stats.max_range_nm = (uint32_t)a->dist_nm;
    }
}

/* ------------------------------------------------------------------------- */
/* Position                                                                  */
/* ------------------------------------------------------------------------- */

static void trail_add(aircraft_t *a, uint32_t t)
{
    if (a->trail_count) {
        int last = (a->trail_head + AC_TRAIL_LEN - 1) % AC_TRAIL_LEN;
        trail_pt_t *p = &a->trail[last];
        /* thin the trail: keep a point every 5 s or when track changes noticeably */
        if (t - p->t_ms < 5000 && dist_nm(p->lat, p->lon, a->lat, a->lon) < 0.5) return;
    }
    trail_pt_t *n = &a->trail[a->trail_head];
    n->lat = (float)a->lat;
    n->lon = (float)a->lon;
    n->alt_ft = a->alt_baro_ms ? a->alt_baro_ft : a->alt_geom_ft;
    n->t_ms = t;
    a->trail_head = (a->trail_head + 1) % AC_TRAIL_LEN;
    if (a->trail_count < AC_TRAIL_LEN) a->trail_count++;
}

static bool position_plausible(aircraft_t *a, double lat, double lon, uint32_t t)
{
    const app_settings_t *cfg = settings_get();
    if (cfg->rx_pos_valid && dist_nm(cfg->rx_lat, cfg->rx_lon, lat, lon) > MAX_RANGE_NM) return false;
    if (a->pos_valid && (t - a->pos_ms) < 120000) {
        double d = dist_nm(a->lat, a->lon, lat, lon);
        double dt_h = (t - a->pos_ms) / 3600000.0 + (2.0 / 3600.0);  /* + slack */
        double max_kt = a->on_ground ? 150.0 : 1500.0;
        if (d / dt_h > max_kt) return false;
    }
    return true;
}

static void set_position(aircraft_t *a, double lat, double lon, bool surface, uint8_t nic, uint32_t t)
{
    a->lat = lat;
    a->lon = lon;
    a->pos_ms = t;
    a->pos_valid = true;
    a->pos_surface = surface;
    a->nic = nic;
    update_distance(a);
    trail_add(a, t);
}

static void handle_cpr(aircraft_t *a, const mm_t *mm, uint32_t t)
{
    const app_settings_t *cfg = settings_get();
    bool surf = mm->cpr_surface;

    if (mm->cpr_odd) {
        a->cpr_odd_lat = mm->cpr_lat; a->cpr_odd_lon = mm->cpr_lon;
        a->cpr_odd_ms = t; a->cpr_odd_surface = surf;
    } else {
        a->cpr_even_lat = mm->cpr_lat; a->cpr_even_lon = mm->cpr_lon;
        a->cpr_even_ms = t; a->cpr_even_surface = surf;
    }

    double lat, lon;
    bool ok = false;
    uint32_t pair_max = surf ? CPR_PAIR_MAX_MS_SURF : CPR_PAIR_MAX_MS_AIR;
    bool have_pair = a->cpr_even_ms && a->cpr_odd_ms &&
                     a->cpr_even_surface == surf && a->cpr_odd_surface == surf &&
                     (uint32_t)abs((int32_t)(a->cpr_even_ms - a->cpr_odd_ms)) <= pair_max;

    /* reference for surface decoding / local decoding */
    bool have_ref = false;
    double ref_lat = 0, ref_lon = 0;
    if (a->pos_valid && (t - a->pos_ms) < 30000) {
        have_ref = true; ref_lat = a->lat; ref_lon = a->lon;
    } else if (cfg->rx_pos_valid) {
        have_ref = true; ref_lat = cfg->rx_lat; ref_lon = cfg->rx_lon;
    }

    if (have_pair) {
        if (surf) {
            if (have_ref) {
                ok = cpr_decode_global_surface(ref_lat, ref_lon,
                                               a->cpr_even_lat, a->cpr_even_lon,
                                               a->cpr_odd_lat, a->cpr_odd_lon,
                                               mm->cpr_odd, &lat, &lon);
            }
        } else {
            ok = cpr_decode_global_airborne(a->cpr_even_lat, a->cpr_even_lon,
                                            a->cpr_odd_lat, a->cpr_odd_lon,
                                            mm->cpr_odd, &lat, &lon);
        }
        if (ok && !position_plausible(a, lat, lon, t)) ok = false;
    }

    /* single-message local decode relative to the aircraft's own recent position */
    if (!ok && a->pos_valid && (t - a->pos_ms) < 30000) {
        ok = cpr_decode_local(a->lat, a->lon, mm->cpr_lat, mm->cpr_lon, mm->cpr_odd, surf, &lat, &lon);
        if (ok && !position_plausible(a, lat, lon, t)) ok = false;
    }

    if (ok) set_position(a, lat, lon, surf, mm->nic_tc, t);
}

/* ------------------------------------------------------------------------- */
/* Apply a decoded message                                                   */
/* ------------------------------------------------------------------------- */

static ac_source_t source_from_mm(const mm_t *mm)
{
    if (mm->df == 17) return SRC_ADSB;
    if (mm->df == 18) {
        switch (mm->addr_type) {
        case ADDR_ADSR_ICAO: return SRC_ADSR;
        case ADDR_TISB_ICAO:
        case ADDR_TISB_OTHER: return SRC_TISB;
        default: return SRC_ADSB_NT;
        }
    }
    return SRC_MODE_S;
}

static int source_rank(ac_source_t s)
{
    switch (s) {
    case SRC_ADSB: return 6;
    case SRC_ADSB_NT: return 5;
    case SRC_ADSR: return 4;
    case SRC_TISB: return 3;
    case SRC_SBS: return 2;
    case SRC_MODE_S: return 1;
    default: return 0;
    }
}

static void derive(aircraft_t *a, uint32_t t)
{
    /* OAT from TAS and Mach: a = TAS/M, T = (a / 38.967854)^2 */
    if (a->tas_ms && a->mach_ms && t - a->tas_ms < FIELD_FRESH_MS && t - a->mach_ms < FIELD_FRESH_MS &&
        a->mach > 0.1f && a->tas_kt > 50.0f) {
        float sos = a->tas_kt / a->mach;
        float tk = (sos / 38.967854f) * (sos / 38.967854f);
        float c = tk - 273.15f;
        if (c > -90.0f && c < 60.0f) {
            a->oat_c = c;
            a->oat_ms = t;
        }
    }
    /* Wind only when heading is known to be true (magnetic would need a declination model) */
    if (a->heading_true && a->hrd_valid &&
        a->tas_ms && a->heading_ms && a->gs_ms && a->track_ms &&
        t - a->tas_ms < FIELD_FRESH_MS && t - a->heading_ms < FIELD_FRESH_MS &&
        t - a->gs_ms < FIELD_FRESH_MS && t - a->track_ms < FIELD_FRESH_MS && !a->on_ground) {
        double gx = a->gs_kt * sin(deg2rad(a->track_deg)), gy = a->gs_kt * cos(deg2rad(a->track_deg));
        double ax = a->tas_kt * sin(deg2rad(a->heading_deg)), ay = a->tas_kt * cos(deg2rad(a->heading_deg));
        double wx = gx - ax, wy = gy - ay;
        a->wind_kt = (float)sqrt(wx * wx + wy * wy);
        double from = atan2(-wx, -wy) * 180.0 / M_PI;
        a->wind_dir = (float)(from < 0 ? from + 360.0 : from);
        a->wind_ms = t;
    }
}

static void apply(aircraft_t *a, const mm_t *mm, ac_source_t src, uint32_t t)
{
    a->last_seen_ms = t;
    a->msgs++;
    a->rate_window_msgs++;
    if (source_rank(src) > source_rank(a->source)) a->source = src;

    if (!isnan(mm->rssi_dbfs)) {
        a->rssi = isnan(a->rssi) ? mm->rssi_dbfs : a->rssi * 0.8f + mm->rssi_dbfs * 0.2f;
    }

    if (mm->callsign_valid) {
        /* a new callsign invalidates the route lookup */
        if (strcmp(a->callsign, mm->callsign) != 0 && a->lookup == LOOKUP_DONE &&
            strcmp(a->lookup_callsign, mm->callsign) != 0) {
            a->lookup = LOOKUP_NONE;
        }
        strlcpy(a->callsign, mm->callsign, sizeof(a->callsign));
        a->callsign_ms = t;
    }
    if (mm->category_valid) { a->category = mm->category; a->category_valid = true; }
    if (mm->squawk_valid) {
        a->squawk = mm->squawk;
        a->squawk_ms = t;
    }
    if (mm->alt_valid) {
        if (mm->alt_src == ALT_SRC_BARO) { a->alt_baro_ft = mm->alt_ft; a->alt_baro_ms = t; }
        else { a->alt_geom_ft = mm->alt_ft; a->alt_geom_ms = t; }
    }
    if (mm->gnss_baro_diff_valid) {
        a->gnss_baro_diff_ft = mm->gnss_baro_diff_ft;
        a->gnss_baro_diff_valid = true;
        if (a->alt_baro_ms && t - a->alt_baro_ms < FIELD_FRESH_MS) {
            a->alt_geom_ft = a->alt_baro_ft + mm->gnss_baro_diff_ft;
            a->alt_geom_ms = t;
        }
    }
    if (mm->ground_valid) { a->on_ground = mm->on_ground; a->ground_valid = true; }
    if (mm->fs_valid) { a->alert = mm->alert; a->spi = mm->spi; }

    if (mm->gs_valid)    { a->gs_kt = mm->gs_kt; a->gs_ms = t; }
    if (mm->track_valid) { a->track_deg = mm->track_deg; a->track_ms = t; }
    if (mm->heading_valid) {
        a->heading_deg = mm->heading_deg; a->heading_ms = t;
        if (!a->hrd_valid) a->heading_true = false;
    }
    if (mm->ias_valid)   { a->ias_kt = mm->ias_kt; a->ias_ms = t; }
    if (mm->tas_valid)   { a->tas_kt = mm->tas_kt; a->tas_ms = t; }
    if (mm->mach_valid)  { a->mach = mm->mach; a->mach_ms = t; }
    if (mm->roll_valid)  { a->roll_deg = mm->roll_deg; a->roll_ms = t; }
    if (mm->track_rate_valid) { a->track_rate = mm->track_rate_dps; a->track_rate_ms = t; }
    if (mm->baro_rate_valid)  { a->baro_rate_fpm = mm->baro_rate_fpm; a->baro_rate_ms = t; }
    if (mm->geom_rate_valid)  { a->geom_rate_fpm = mm->geom_rate_fpm; a->geom_rate_ms = t; }
    if (mm->nac_v_valid) { a->nac_v = mm->nac_v; a->nac_v_valid = true; }

    if (mm->emergency_valid) { a->emergency = mm->emergency; a->emergency_valid = true; }
    if (mm->acas_ra_valid)   { a->acas_ra_active = mm->acas_ra_active; a->acas_ra_ms = t; }

    if (mm->sel_alt_valid) { a->sel_alt_ft = mm->sel_alt_ft; a->sel_alt_ms = t; a->sel_alt_fms = mm->sel_alt_fms; }
    if (mm->fms_alt_valid) { a->fms_alt_ft = mm->fms_alt_ft; a->fms_alt_ms = t; }
    if (mm->qnh_valid)     { a->qnh_hpa = mm->qnh_hpa; a->qnh_ms = t; }
    if (mm->sel_hdg_valid) { a->sel_hdg_deg = mm->sel_hdg_deg; a->sel_hdg_ms = t; }
    if (mm->nav_modes_valid) {
        a->nav_modes_valid = true; a->nav_modes_ms = t;
        a->ap_on = mm->ap_on; a->vnav_on = mm->vnav_on; a->alt_hold_on = mm->alt_hold_on;
        a->approach_on = mm->approach_on; a->lnav_on = mm->lnav_on; a->tcas_on = mm->tcas_on;
    }

    if (mm->opstatus_valid) {
        a->adsb_version = mm->adsb_version; a->adsb_version_valid = true;
        a->tcas_on = a->tcas_on || mm->tcas_ra_active;
        a->ident_active = mm->ident_active;
        if (mm->tcas_ra_active) { a->acas_ra_active = true; a->acas_ra_ms = t; }
        if (mm->hrd_valid) { a->hrd_valid = true; a->heading_true = mm->heading_is_true; }
        if (mm->acas_op_valid) { a->acas_operational = mm->acas_operational; a->acas_op_valid = true; }
        if (mm->cap_valid) { a->es_in = mm->es_in; a->uat_in = mm->uat_in; a->cap_valid = true; }
        if (mm->len_width_valid) { a->len_width_code = mm->len_width_code; a->len_width_valid = true; }
        if (mm->sda_valid) { a->sda = mm->sda; a->sda_valid = true; }
        if (mm->gva_valid) { a->gva = mm->gva; a->gva_valid = true; }
    }
    if (mm->nac_p_valid)    { a->nac_p = mm->nac_p; a->nac_p_valid = true; }
    if (mm->sil_valid)      { a->sil = mm->sil; a->sil_valid = true; }
    if (mm->nic_baro_valid) { a->nic_baro = mm->nic_baro; a->nic_baro_valid = true; }

    if (mm->bds >= 0x10 && mm->bds <= 0x60 && (mm->bds & 0x0F) == 0) {
        a->bds_seen |= (uint8_t)(1u << ((mm->bds >> 4) - 1));
    }

    if (mm->cpr_valid) handle_cpr(a, mm, t);

    derive(a, t);
}

void aircraft_ingest_frame(const uint8_t *frame, int len, float rssi_dbfs, uint64_t rx_us)
{
    mm_t mm;
    uint32_t t = now_ms();

    aircraft_lock();
    s_stats.msgs_total++;
    s_rate_msgs++;
    if (!modes_decode(frame, len, &mm, known_addr_cb)) {
        s_stats.msgs_bad++;
        aircraft_unlock();
        return;
    }
    mm.rssi_dbfs = rssi_dbfs;
    mm.rx_time_us = rx_us;
    s_stats.msgs_good++;
    s_stats.df_count[mm.df]++;

    aircraft_t *a = aircraft_find(mm.addr);
    if (!a) {
        /* only verified-CRC messages may create an aircraft */
        if (!mm.crc_ok) { aircraft_unlock(); return; }
        a = aircraft_create(mm.addr);
    }

    if (mm.df == 20 || mm.df == 21) {
        modes_ref_t ref = {
            .gs_valid = a->gs_ms && t - a->gs_ms < 30000, .gs_kt = a->gs_kt,
            .track_valid = a->track_ms && t - a->track_ms < 30000, .track_deg = a->track_deg,
            .alt_valid = a->alt_baro_ms && t - a->alt_baro_ms < 30000, .alt_ft = a->alt_baro_ft,
        };
        modes_decode_commb(&mm, &ref);
    }

    apply(a, &mm, source_from_mm(&mm), t);
    aircraft_unlock();
}

void aircraft_ingest_decoded(mm_t *mm, ac_source_t src, bool pos_valid, double lat, double lon)
{
    uint32_t t = now_ms();
    aircraft_lock();
    s_stats.msgs_total++;
    s_stats.msgs_good++;
    s_rate_msgs++;
    aircraft_t *a = aircraft_find(mm->addr);
    if (!a) a = aircraft_create(mm->addr);
    apply(a, mm, src, t);
    if (pos_valid && (lat != 0.0 || lon != 0.0) && position_plausible(a, lat, lon, t)) {
        set_position(a, lat, lon, a->on_ground, 0, t);
    }
    aircraft_unlock();
}

void aircraft_tick(void)
{
    uint32_t t = now_ms();
    aircraft_lock();

    uint32_t dt = t - s_rate_t0;
    if (dt >= 1000) {
        float r = s_rate_msgs * 1000.0f / dt;
        s_stats.msg_rate = s_stats.msg_rate * 0.5f + r * 0.5f;
        s_rate_msgs = 0;
        s_rate_t0 = t;
    }

    for (int i = 0; i < AC_MAX; i++) {
        aircraft_t *a = &s_ac[i];
        if (!a->used) continue;
        if (t - a->last_seen_ms > AC_EXPIRE_MS) {
            a->used = false;
            continue;
        }
        if (a->pos_valid && t - a->pos_ms > AC_POS_STALE_MS * 4) a->pos_valid = false;
        uint32_t w = t - a->rate_window_start_ms;
        if (w >= 5000) {
            a->msg_rate = a->rate_window_msgs * 1000.0f / w;
            a->rate_window_msgs = 0;
            a->rate_window_start_ms = t;
        }
        update_distance(a);
    }
    aircraft_unlock();
}

void aircraft_feed_event(bool connected)
{
    aircraft_lock();
    s_rate_msgs = 0;
    s_rate_t0 = now_ms();
    if (!connected) s_stats.msg_rate = 0;
    aircraft_unlock();
}
