#pragma once

/*
 * Mode S / ADS-B (1090ES) frame decoder.
 *
 * Decodes DF0/4/5/11/16/17/18/20/21 into a flat mm_t ("modes message")
 * with per-field "valid" flags. CPR position decoding needs state across
 * messages, so it lives in aircraft.c; this module only extracts the raw CPR
 * fields.
 */

#include <stdbool.h>
#include <stdint.h>

#define MODES_SHORT_BYTES 7
#define MODES_LONG_BYTES  14

typedef enum {
    ADDR_ICAO = 0,        /* DF17 / DF11 / Comm-B with known ICAO address */
    ADDR_ADSB_OTHER,      /* DF18 CF=1: ADS-B from non-transponder device, anonymous address */
    ADDR_ADSR_ICAO,       /* DF18 CF=6: ADS-R rebroadcast (UAT -> 1090) */
    ADDR_TISB_ICAO,       /* DF18 CF=2: fine TIS-B, ICAO address */
    ADDR_TISB_OTHER,      /* DF18 CF=2/3/5: TIS-B, non-ICAO / track-file address */
    ADDR_MODE_S,          /* Mode S surveillance reply (address from AP parity) */
} addr_type_t;

typedef enum {
    EMERG_NONE = 0,
    EMERG_GENERAL,        /* 7700 */
    EMERG_LIFEGUARD,      /* medical */
    EMERG_MINFUEL,
    EMERG_NORDO,          /* 7600 */
    EMERG_UNLAWFUL,       /* 7500 */
    EMERG_DOWNED,
    EMERG_RESERVED,
} emergency_t;

typedef enum {
    ALT_SRC_BARO = 0,
    ALT_SRC_GNSS,
} alt_src_t;

typedef struct {
    /* --- frame --- */
    uint8_t  msg[MODES_LONG_BYTES];
    uint8_t  len;              /* bytes: 7 or 14 */
    uint8_t  df;
    uint32_t addr;             /* 24-bit address */
    addr_type_t addr_type;
    bool     crc_ok;           /* parity verified (DF11/17/18) */
    bool     addr_from_ap;     /* address recovered from AP: only trust if already known */
    float    rssi_dbfs;        /* NaN if unknown */
    uint64_t rx_time_us;

    /* --- common surveillance fields --- */
    bool     alt_valid;        int32_t alt_ft;  alt_src_t alt_src;
    bool     ground_valid;     bool on_ground;
    bool     squawk_valid;     uint16_t squawk;  /* 4 octal digits packed as hex, e.g. 0x7700 */
    bool     fs_valid;         bool alert; bool spi;
    bool     ca_valid;         uint8_t ca;       /* capability (DF11/17) */

    /* --- identification (TC 1-4 / BDS 2,0) --- */
    bool     callsign_valid;   char callsign[9];
    bool     category_valid;   uint8_t category; /* 0xA0..0xD7 style: set=A..D, code */

    /* --- position (raw CPR) --- */
    bool     cpr_valid;
    bool     cpr_odd;
    bool     cpr_surface;
    uint32_t cpr_lat, cpr_lon; /* 17-bit */
    uint8_t  nic_tc;           /* NIC derived from type code (approx) */

    /* --- velocity (TC 19 / surface movement) --- */
    bool     gs_valid;         float gs_kt;
    bool     track_valid;      float track_deg;
    bool     heading_valid;    float heading_deg;  bool heading_is_true;
    bool     ias_valid;        float ias_kt;
    bool     tas_valid;        float tas_kt;
    bool     mach_valid;       float mach;
    bool     baro_rate_valid;  int32_t baro_rate_fpm;
    bool     geom_rate_valid;  int32_t geom_rate_fpm;
    bool     gnss_baro_diff_valid; int32_t gnss_baro_diff_ft;
    bool     roll_valid;       float roll_deg;
    bool     track_rate_valid; float track_rate_dps;
    uint8_t  nac_v;            bool nac_v_valid;

    /* --- aircraft status (TC 28) --- */
    bool     emergency_valid;  emergency_t emergency;

    /* --- target state & status (TC 29 / BDS 4,0) --- */
    bool     sel_alt_valid;    int32_t sel_alt_ft;   bool sel_alt_fms;
    bool     fms_alt_valid;    int32_t fms_alt_ft;
    bool     qnh_valid;        float qnh_hpa;
    bool     sel_hdg_valid;    float sel_hdg_deg;
    bool     nav_modes_valid;
    bool     ap_on, vnav_on, alt_hold_on, approach_on, lnav_on, tcas_on;

    /* --- operational status (TC 31) --- */
    bool     opstatus_valid;
    uint8_t  adsb_version;
    uint8_t  nac_p;            bool nac_p_valid;
    uint8_t  sil;              bool sil_valid;
    uint8_t  sda;              bool sda_valid;
    uint8_t  gva;              bool gva_valid;
    uint8_t  nic_baro;         bool nic_baro_valid;
    bool     tcas_ra_active;
    bool     acas_op_valid;    bool acas_operational;
    bool     hrd_valid;        /* heading_is_true reliable from opstatus */
    bool     ident_active;
    bool     es_in, uat_in;    bool cap_valid;
    bool     len_width_valid;  uint8_t len_width_code;

    /* --- ACAS (DF0/16, TC28 st2, BDS3,0) --- */
    bool     acas_ra_valid;    bool acas_ra_active;

    /* --- Comm-B extras --- */
    uint8_t  bds;              /* inferred register, e.g. 0x40, 0x50, 0x60; 0 if unknown */

    /* informational */
    uint8_t  me_type;          /* DF17/18 type code */
    uint8_t  me_subtype;
} mm_t;

/* CRC-24 of `len` bytes (parity over all but the last 3 bytes, xor'ed with them) */
uint32_t modes_crc_residual(const uint8_t *msg, int len_bytes);

/*
 * Decode one frame (7 or 14 bytes). Returns false if the frame is unusable.
 * `known_addr` lets the decoder accept AP-parity messages only for aircraft
 * already seen with a verified CRC.
 */
bool modes_decode(const uint8_t *frame, int len_bytes, mm_t *mm,
                  bool (*known_addr)(uint32_t addr));

/* Decoder hint for Comm-B BDS 5,0 vs 6,0 disambiguation (may be NULL fields) */
typedef struct {
    bool gs_valid;    float gs_kt;
    bool track_valid; float track_deg;
    bool alt_valid;   int32_t alt_ft;
} modes_ref_t;

/* Re-run Comm-B (DF20/21) inference with aircraft context. Called by aircraft.c. */
void modes_decode_commb(mm_t *mm, const modes_ref_t *ref);

/* CPR helpers */
int  cpr_nl(double lat);
bool cpr_decode_global_airborne(uint32_t even_lat, uint32_t even_lon,
                                uint32_t odd_lat, uint32_t odd_lon,
                                bool use_odd, double *lat, double *lon);
bool cpr_decode_global_surface(double ref_lat, double ref_lon,
                               uint32_t even_lat, uint32_t even_lon,
                               uint32_t odd_lat, uint32_t odd_lon,
                               bool use_odd, double *lat, double *lon);
bool cpr_decode_local(double ref_lat, double ref_lon, uint32_t cpr_lat, uint32_t cpr_lon,
                      bool odd, bool surface, double *lat, double *lon);

const char *emergency_str(emergency_t e);
const char *category_str(uint8_t category);   /* e.g. "Heavy (>300k lb)" */
