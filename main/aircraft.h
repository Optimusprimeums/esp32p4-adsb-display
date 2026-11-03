#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "modes.h"

#define AC_MAX          400
#define AC_TRAIL_LEN    240     /* ~20 min at one point per 5 s */
#define AC_EXPIRE_MS    60000     /* drop aircraft not heard for this long */
#define AC_POS_STALE_MS 30000

typedef enum {
    SRC_UNKNOWN = 0,
    SRC_MODE_S,      /* surveillance replies only */
    SRC_ADSB,        /* DF17 */
    SRC_ADSB_NT,     /* DF18 non-transponder */
    SRC_ADSR,
    SRC_TISB,
    SRC_SBS,         /* pre-decoded SBS feed */
} ac_source_t;

typedef enum {
    LOOKUP_NONE = 0,
    LOOKUP_PENDING,
    LOOKUP_DONE,
    LOOKUP_NOT_FOUND,
    LOOKUP_FAILED,
} lookup_state_t;

typedef struct {
    float lat, lon;
    int32_t alt_ft;
    uint32_t t_ms;
} trail_pt_t;

typedef struct {
    char name[64];
    char icao[5];
    char iata[4];
    char city[32];
    char country[32];
    double lat, lon;
    bool valid;
} airport_t;

typedef struct {
    bool used;
    uint32_t addr;              /* bit 24 set => non-ICAO address */
    ac_source_t source;

    uint32_t first_seen_ms, last_seen_ms;
    uint32_t msgs;
    float    rssi;              /* smoothed dBFS, NaN if unknown */
    float    msg_rate;          /* messages/s, smoothed */
    uint32_t rate_window_start_ms, rate_window_msgs;

    char     callsign[9];       uint32_t callsign_ms;
    uint8_t  category;          bool category_valid;
    uint16_t squawk;            uint32_t squawk_ms;

    int32_t  alt_baro_ft;       uint32_t alt_baro_ms;
    int32_t  alt_geom_ft;       uint32_t alt_geom_ms;
    int32_t  gnss_baro_diff_ft; bool gnss_baro_diff_valid;
    bool     on_ground;         bool ground_valid;

    double   lat, lon;          uint32_t pos_ms;   bool pos_valid;
    bool     pos_surface;
    uint8_t  nic;

    float    gs_kt;             uint32_t gs_ms;
    float    track_deg;         uint32_t track_ms;
    float    heading_deg;       uint32_t heading_ms;  bool heading_true; bool hrd_valid;
    float    ias_kt;            uint32_t ias_ms;
    float    tas_kt;            uint32_t tas_ms;
    float    mach;              uint32_t mach_ms;
    float    roll_deg;          uint32_t roll_ms;
    float    track_rate;        uint32_t track_rate_ms;
    int32_t  baro_rate_fpm;     uint32_t baro_rate_ms;
    int32_t  geom_rate_fpm;     uint32_t geom_rate_ms;

    /* derived */
    float    oat_c;             uint32_t oat_ms;    /* from TAS & Mach */
    float    wind_dir, wind_kt; uint32_t wind_ms;   /* from GS/track vs TAS/heading */
    double   dist_nm;           float bearing_deg;  bool dist_valid;

    /* intent / autopilot */
    int32_t  sel_alt_ft;        uint32_t sel_alt_ms;  bool sel_alt_fms;
    int32_t  fms_alt_ft;        uint32_t fms_alt_ms;
    float    qnh_hpa;           uint32_t qnh_ms;
    float    sel_hdg_deg;       uint32_t sel_hdg_ms;
    bool     nav_modes_valid;   uint32_t nav_modes_ms;
    bool     ap_on, vnav_on, alt_hold_on, approach_on, lnav_on, tcas_on;

    /* status */
    emergency_t emergency;      bool emergency_valid;
    bool     alert, spi;
    bool     acas_ra_active;    uint32_t acas_ra_ms;
    bool     acas_operational;  bool acas_op_valid;
    bool     ident_active;

    /* quality / version */
    bool     adsb_version_valid; uint8_t adsb_version;
    uint8_t  nac_p, nac_v, sil, sda, gva, nic_baro;
    bool     nac_p_valid, nac_v_valid, sil_valid, sda_valid, gva_valid, nic_baro_valid;
    bool     es_in, uat_in, cap_valid;
    uint8_t  len_width_code;    bool len_width_valid;

    /* Comm-B registers seen (bitmask, 1<<n for BDS n,0 n=1..6) */
    uint8_t  bds_seen;

    /* CPR state */
    uint32_t cpr_even_lat, cpr_even_lon, cpr_odd_lat, cpr_odd_lon;
    uint32_t cpr_even_ms, cpr_odd_ms;
    bool     cpr_even_surface, cpr_odd_surface;

    /* trail */
    trail_pt_t trail[AC_TRAIL_LEN];
    uint16_t trail_head, trail_count;

    /* online enrichment */
    lookup_state_t lookup;
    uint32_t lookup_ms;
    char     lookup_callsign[9];  /* callsign used for the route lookup */
    char     registration[16];
    char     type_code[8];
    char     type_name[48];
    char     manufacturer[40];
    char     owner[64];
    char     owner_country[40];
    char     airline[48];
    char     airline_callsign[24];
    airport_t origin, destination;

    /* derived from address */
    const char *country;        /* from ICAO block allocation */
    bool     military;
} aircraft_t;

typedef struct {
    uint32_t msgs_total;
    uint32_t msgs_good;
    uint32_t msgs_bad;
    float    msg_rate;         /* per second */
    uint32_t max_range_nm;
    uint32_t aircraft_total;   /* distinct addresses since boot */
    uint32_t df_count[25];
} feed_stats_t;

void aircraft_init(void);

/* Ingest a raw Mode S frame (from Beast/AVR) */
void aircraft_ingest_frame(const uint8_t *frame, int len, float rssi_dbfs, uint64_t rx_us);
/* Ingest a pre-decoded message (SBS) — fields marked valid are applied */
void aircraft_ingest_decoded(mm_t *mm, ac_source_t src, bool pos_valid, double lat, double lon);

void aircraft_lock(void);
void aircraft_unlock(void);

/* Must be called with the lock held. Returns pointer into table. */
aircraft_t *aircraft_table(int *count);
aircraft_t *aircraft_find(uint32_t addr);

feed_stats_t *aircraft_stats(void);

/* Periodic maintenance: expire, recompute distance, rates. Call ~1 Hz. */
void aircraft_tick(void);

/* Mark feed (re)connect so stats reset sensibly */
void aircraft_feed_event(bool connected);

uint32_t now_ms(void);
