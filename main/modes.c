#include "modes.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Bit helpers                                                               */
/* ------------------------------------------------------------------------- */

/* Bits numbered from 1 (MSB of first byte), inclusive range, as in the
 * standard documents ("The 1090MHz Riddle" / DO-260B). */
static uint32_t getbits(const uint8_t *m, int first, int last)
{
    uint32_t v = 0;
    for (int b = first; b <= last; b++) {
        int byte = (b - 1) >> 3;
        int bit = 7 - ((b - 1) & 7);
        v = (v << 1) | ((m[byte] >> bit) & 1);
    }
    return v;
}

/* ME / MB field (56 bits starting at message bit 33), 1-indexed within ME */
static inline uint32_t me(const uint8_t *m, int first, int last)
{
    return getbits(m, 32 + first, 32 + last);
}

/* MB field, 0-indexed with exclusive end (matches pyModeS conventions) */
static inline uint32_t mb0(const uint8_t *m, int start, int end_excl)
{
    return getbits(m, 33 + start, 32 + end_excl);
}

/* ------------------------------------------------------------------------- */
/* CRC                                                                       */
/* ------------------------------------------------------------------------- */

static uint32_t s_crc_table[256];
static bool s_crc_ready;

static void crc_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint32_t c = (uint32_t)i << 16;
        for (int j = 0; j < 8; j++) {
            c = (c & 0x800000) ? ((c << 1) ^ 0xFFF409) : (c << 1);
        }
        s_crc_table[i] = c & 0xFFFFFF;
    }
    s_crc_ready = true;
}

uint32_t modes_crc_residual(const uint8_t *msg, int len_bytes)
{
    if (!s_crc_ready) crc_init();
    uint32_t crc = 0;
    for (int i = 0; i < len_bytes - 3; i++) {
        crc = (crc << 8) ^ s_crc_table[((crc >> 16) ^ msg[i]) & 0xFF];
        crc &= 0xFFFFFF;
    }
    uint32_t tail = ((uint32_t)msg[len_bytes - 3] << 16) | ((uint32_t)msg[len_bytes - 2] << 8) | msg[len_bytes - 1];
    return crc ^ tail;
}

/* ------------------------------------------------------------------------- */
/* Field decoders                                                            */
/* ------------------------------------------------------------------------- */

/* 13-bit ID/AC field -> Mode A "hex gillham" layout 0xABCD (A4A2A1, B4B2B1, ...) */
static uint32_t id13_to_hexgillham(uint32_t f)
{
    uint32_t h = 0;
    if (f & 0x1000) h |= 0x0010; /* C1 */
    if (f & 0x0800) h |= 0x1000; /* A1 */
    if (f & 0x0400) h |= 0x0020; /* C2 */
    if (f & 0x0200) h |= 0x2000; /* A2 */
    if (f & 0x0100) h |= 0x0040; /* C4 */
    if (f & 0x0080) h |= 0x4000; /* A4 */
    if (f & 0x0020) h |= 0x0100; /* B1 */
    if (f & 0x0010) h |= 0x0001; /* D1 / Q */
    if (f & 0x0008) h |= 0x0200; /* B2 */
    if (f & 0x0004) h |= 0x0002; /* D2 */
    if (f & 0x0002) h |= 0x0400; /* B4 */
    if (f & 0x0001) h |= 0x0004; /* D4 */
    return h;
}

/* Gillham (Gray) Mode C -> altitude in 100 ft units, or INT32_MIN if invalid */
static int32_t gillham_to_100ft(uint32_t a)
{
    uint32_t five = 0, one = 0;
    if ((a & 0xFFFF8889) != 0 || (a & 0x000000F0) == 0) return INT32_MIN;
    if (a & 0x0010) one ^= 0x007; /* C1 */
    if (a & 0x0020) one ^= 0x003; /* C2 */
    if (a & 0x0040) one ^= 0x001; /* C4 */
    if ((one & 5) == 5) one ^= 2; /* 7 <-> 5 */
    if (one > 5) return INT32_MIN;
    if (a & 0x0002) five ^= 0x0FF; /* D2 */
    if (a & 0x0004) five ^= 0x07F; /* D4 */
    if (a & 0x1000) five ^= 0x03F; /* A1 */
    if (a & 0x2000) five ^= 0x01F; /* A2 */
    if (a & 0x4000) five ^= 0x00F; /* A4 */
    if (a & 0x0100) five ^= 0x007; /* B1 */
    if (a & 0x0200) five ^= 0x003; /* B2 */
    if (a & 0x0400) five ^= 0x001; /* B4 */
    if (five & 1) one = 6 - one;
    return (int32_t)(five * 5 + one) - 13;
}

/* AC13 (DF0/4/16/20) -> feet; returns false if no/invalid altitude */
static bool decode_ac13(uint32_t ac, int32_t *ft)
{
    if (ac == 0) return false;
    bool m_bit = ac & 0x0040;
    bool q_bit = ac & 0x0010;
    if (m_bit) {
        return false;  /* metric altitude: essentially never used */
    }
    if (q_bit) {
        uint32_t n = ((ac & 0x1F80) >> 2) | ((ac & 0x0020) >> 1) | (ac & 0x000F);
        *ft = (int32_t)n * 25 - 1000;
        return true;
    }
    int32_t h = gillham_to_100ft(id13_to_hexgillham(ac));
    if (h == INT32_MIN || h < -12) return false;
    *ft = h * 100;
    return true;
}

/* AC12 (ADS-B airborne position) -> feet */
static bool decode_ac12(uint32_t ac, int32_t *ft)
{
    if (ac == 0) return false;
    if (ac & 0x0010) {  /* Q bit */
        uint32_t n = ((ac & 0x0FE0) >> 1) | (ac & 0x000F);
        *ft = (int32_t)n * 25 - 1000;
        return true;
    }
    /* insert M=0 to get AC13 layout, then Gillham */
    uint32_t ac13 = ((ac & 0x0FC0) << 1) | (ac & 0x003F);
    return decode_ac13(ac13, ft);
}

static const char s_ais_charset[] =
    "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";

/* 8 x 6-bit chars starting at ME/MB bit 9 (1-indexed). Returns false on junk. */
static bool decode_callsign(const uint8_t *m, char out[9])
{
    int nonspace = 0;
    for (int i = 0; i < 8; i++) {
        uint32_t c = me(m, 9 + i * 6, 14 + i * 6);
        char ch = s_ais_charset[c & 0x3F];
        if (ch == '#') return false;
        out[i] = ch;
        if (ch != ' ') nonspace++;
    }
    out[8] = 0;
    /* trim trailing spaces */
    for (int i = 7; i >= 0 && out[i] == ' '; i--) out[i] = 0;
    return nonspace > 0;
}

static float surface_movement_kt(uint32_t mov, bool *valid)
{
    *valid = true;
    if (mov == 0 || mov >= 125) { *valid = false; return 0; }
    if (mov == 1) return 0.0f;
    if (mov <= 8)   return 0.125f + (mov - 2) * 0.125f;
    if (mov <= 12)  return 1.0f + (mov - 9) * 0.25f;
    if (mov <= 38)  return 2.0f + (mov - 13) * 0.5f;
    if (mov <= 93)  return 15.0f + (mov - 39) * 1.0f;
    if (mov <= 108) return 70.0f + (mov - 94) * 2.0f;
    if (mov <= 123) return 100.0f + (mov - 109) * 5.0f;
    return 175.0f;  /* 124: >= 175 kt */
}

/* ------------------------------------------------------------------------- */
/* DF17/18 extended squitter                                                 */
/* ------------------------------------------------------------------------- */

static void decode_es(const uint8_t *m, mm_t *mm)
{
    uint8_t tc = me(m, 1, 5);
    uint8_t st = me(m, 6, 8);
    mm->me_type = tc;
    mm->me_subtype = st;

    if (tc >= 1 && tc <= 4) {
        /* identification and category */
        static const uint8_t set_for_tc[5] = {0, 0xD, 0xC, 0xB, 0xA};
        mm->category = (uint8_t)((set_for_tc[tc] << 4) | st);
        mm->category_valid = true;
        mm->callsign_valid = decode_callsign(m, mm->callsign);
        return;
    }

    if (tc >= 5 && tc <= 8) {
        /* surface position */
        bool v;
        float gs = surface_movement_kt(me(m, 6, 12), &v);
        if (v) { mm->gs_valid = true; mm->gs_kt = gs; }
        if (me(m, 13, 13)) {
            mm->track_valid = true;
            mm->track_deg = me(m, 14, 20) * 360.0f / 128.0f;
        }
        mm->cpr_valid = true;
        mm->cpr_surface = true;
        mm->cpr_odd = me(m, 22, 22);
        mm->cpr_lat = me(m, 23, 39);
        mm->cpr_lon = me(m, 40, 56);
        mm->ground_valid = true;
        mm->on_ground = true;
        mm->nic_tc = (tc == 5) ? 11 : (tc == 6) ? 10 : (tc == 7) ? 8 : 0;
        return;
    }

    if ((tc >= 9 && tc <= 18) || (tc >= 20 && tc <= 22)) {
        /* airborne position */
        uint8_t ss = me(m, 6, 7);
        if (ss == 1 || ss == 2) { mm->fs_valid = true; mm->alert = true; }
        if (ss == 3) { mm->fs_valid = true; mm->spi = true; }

        uint32_t ac = me(m, 9, 20);
        if (tc <= 18) {
            int32_t ft;
            if (decode_ac12(ac, &ft)) {
                mm->alt_valid = true; mm->alt_ft = ft; mm->alt_src = ALT_SRC_BARO;
            }
        } else if (ac != 0) {
            /* GNSS height (HAE) in metres */
            mm->alt_valid = true;
            mm->alt_ft = (int32_t)lroundf(ac * 3.28084f);
            mm->alt_src = ALT_SRC_GNSS;
        }
        uint32_t lat = me(m, 23, 39), lon = me(m, 40, 56);
        if (lat || lon) {
            mm->cpr_valid = true;
            mm->cpr_surface = false;
            mm->cpr_odd = me(m, 22, 22);
            mm->cpr_lat = lat;
            mm->cpr_lon = lon;
        }
        mm->ground_valid = true;
        mm->on_ground = false;
        static const uint8_t nic_by_tc[23] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            11, 10, 8, 7, 6, 5, 4, 2, 1, 0,  /* 9..18 */
            0, 11, 10, 0                       /* 19..22 */
        };
        mm->nic_tc = nic_by_tc[tc];
        return;
    }

    if (tc == 19) {
        /* airborne velocity */
        if (st < 1 || st > 4) return;
        mm->nac_v = me(m, 11, 13);
        mm->nac_v_valid = true;
        int mult = (st == 2 || st == 4) ? 4 : 1;  /* supersonic */

        if (st == 1 || st == 2) {
            uint32_t vew = me(m, 15, 24), vns = me(m, 26, 35);
            if (vew && vns) {
                float ew = (float)((int)vew - 1) * mult * (me(m, 14, 14) ? -1 : 1);
                float ns = (float)((int)vns - 1) * mult * (me(m, 25, 25) ? -1 : 1);
                mm->gs_valid = true;
                mm->gs_kt = sqrtf(ew * ew + ns * ns);
                if (mm->gs_kt > 0.1f) {
                    float trk = atan2f(ew, ns) * 180.0f / (float)M_PI;
                    if (trk < 0) trk += 360.0f;
                    mm->track_valid = true;
                    mm->track_deg = trk;
                }
            }
        } else {
            if (me(m, 14, 14)) {
                mm->heading_valid = true;
                mm->heading_deg = me(m, 15, 24) * 360.0f / 1024.0f;
                mm->heading_is_true = false;  /* refined by HRD in opstatus */
            }
            uint32_t as = me(m, 26, 35);
            if (as) {
                float kt = (float)((int)as - 1) * mult;
                if (me(m, 25, 25)) { mm->tas_valid = true; mm->tas_kt = kt; }
                else { mm->ias_valid = true; mm->ias_kt = kt; }
            }
        }

        uint32_t vr = me(m, 38, 46);
        if (vr) {
            int32_t fpm = ((int32_t)vr - 1) * 64 * (me(m, 37, 37) ? -1 : 1);
            if (me(m, 36, 36)) { mm->baro_rate_valid = true; mm->baro_rate_fpm = fpm; }
            else { mm->geom_rate_valid = true; mm->geom_rate_fpm = fpm; }
        }
        uint32_t dalt = me(m, 50, 56);
        if (dalt) {
            mm->gnss_baro_diff_valid = true;
            mm->gnss_baro_diff_ft = ((int32_t)dalt - 1) * 25 * (me(m, 49, 49) ? -1 : 1);
        }
        return;
    }

    if (tc == 28) {
        if (st == 1) {
            mm->emergency_valid = true;
            mm->emergency = (emergency_t)me(m, 9, 11);
            uint32_t id = me(m, 12, 24);
            if (id) {
                mm->squawk_valid = true;
                mm->squawk = (uint16_t)id13_to_hexgillham(id);
            }
        } else if (st == 2) {
            uint32_t ara = me(m, 9, 22);
            uint32_t rat = me(m, 27, 27);
            mm->acas_ra_valid = true;
            mm->acas_ra_active = (ara & 0x2000) && !rat;
        }
        return;
    }

    if (tc == 29) {
        if (me(m, 6, 7) != 1) return;   /* only the DO-260B (v2) layout */
        uint32_t alt = me(m, 10, 20);
        if (alt) {
            mm->sel_alt_valid = true;
            mm->sel_alt_ft = ((int32_t)alt - 1) * 32;
            mm->sel_alt_fms = me(m, 9, 9);
        }
        uint32_t baro = me(m, 21, 29);
        if (baro) {
            mm->qnh_valid = true;
            mm->qnh_hpa = 800.0f + ((int)baro - 1) * 0.8f;
        }
        if (me(m, 30, 30)) {
            mm->sel_hdg_valid = true;
            mm->sel_hdg_deg = me(m, 31, 39) * 180.0f / 256.0f;
        }
        mm->nac_p = me(m, 40, 43); mm->nac_p_valid = true;
        mm->nic_baro = me(m, 44, 44); mm->nic_baro_valid = true;
        mm->sil = me(m, 45, 46); mm->sil_valid = true;
        if (me(m, 47, 47)) {
            mm->nav_modes_valid = true;
            mm->ap_on = me(m, 48, 48);
            mm->vnav_on = me(m, 49, 49);
            mm->alt_hold_on = me(m, 50, 50);
            mm->approach_on = me(m, 52, 52);
            mm->tcas_on = me(m, 53, 53);
            mm->lnav_on = me(m, 54, 54);
        }
        return;
    }

    if (tc == 31) {
        if (st > 1) return;
        mm->opstatus_valid = true;
        mm->adsb_version = me(m, 41, 43);
        if (mm->adsb_version >= 1) {
            mm->nac_p = me(m, 45, 48); mm->nac_p_valid = true;
            mm->sil = me(m, 51, 52);   mm->sil_valid = true;
            mm->tcas_ra_active = me(m, 27, 27);
            mm->ident_active = me(m, 28, 28);
            if (st == 0) {
                mm->cap_valid = true;
                mm->acas_op_valid = true;
                mm->acas_operational = me(m, 11, 11);
                mm->es_in = me(m, 12, 12);
                mm->uat_in = me(m, 19, 19);
                mm->gva = me(m, 49, 50);   mm->gva_valid = true;
                mm->nic_baro = me(m, 53, 53); mm->nic_baro_valid = true;
            } else {
                mm->cap_valid = true;
                mm->es_in = me(m, 12, 12);
                mm->uat_in = me(m, 16, 16);
                mm->len_width_code = me(m, 21, 24);
                mm->len_width_valid = true;
            }
            if (mm->adsb_version >= 2) {
                mm->sda = me(m, 31, 32); mm->sda_valid = true;
                /* HRD: 1 = headings are magnetic, 0 = true */
                mm->heading_is_true = !me(m, 54, 54);
                mm->hrd_valid = true;
            }
        }
        return;
    }
}

/* ------------------------------------------------------------------------- */
/* Comm-B (DF20/21) BDS inference                                            */
/* ------------------------------------------------------------------------- */

/* status bit at idx; if clear then [start,end) must be all zero */
static bool status_ok(const uint8_t *m, int idx, int start, int end)
{
    if (mb0(m, idx, idx + 1)) return true;
    return mb0(m, start, end) == 0;
}

static bool mb_all_zero(const uint8_t *m)
{
    for (int i = 4; i < 11; i++) if (m[i]) return false;
    return true;
}

static bool is_bds10(const uint8_t *m)
{
    return mb0(m, 0, 8) == 0x10 && mb0(m, 9, 14) == 0;
}

static bool is_bds20(const uint8_t *m, char cs[9])
{
    if (mb0(m, 0, 8) != 0x20) return false;
    return decode_callsign(m, cs);
}

static bool is_bds30(const uint8_t *m)
{
    return mb0(m, 0, 8) == 0x30;
}

static bool is_bds40(const uint8_t *m)
{
    if (!status_ok(m, 0, 1, 13)) return false;
    if (!status_ok(m, 13, 14, 26)) return false;
    if (!status_ok(m, 26, 27, 39)) return false;
    if (!status_ok(m, 47, 48, 51)) return false;
    if (!status_ok(m, 53, 54, 56)) return false;
    if (mb0(m, 39, 47) != 0) return false;   /* reserved */
    if (mb0(m, 51, 53) != 0) return false;   /* reserved */
    if (!mb0(m, 0, 1) && !mb0(m, 13, 14) && !mb0(m, 26, 27)) return false;
    if (mb0(m, 0, 1) && mb0(m, 1, 13) * 16 > 50000) return false;
    if (mb0(m, 26, 27)) {
        float qnh = mb0(m, 27, 39) * 0.1f + 800.0f;
        if (qnh < 900.0f || qnh > 1100.0f) return false;
    }
    return true;
}

typedef struct {
    bool roll_v; float roll;
    bool trk_v;  float trk;
    bool gs_v;   float gs;
    bool rate_v; float rate;
    bool tas_v;  float tas;
} bds50_t;

static bool is_bds50(const uint8_t *m, bds50_t *o)
{
    if (!status_ok(m, 0, 1, 11)) return false;
    if (!status_ok(m, 11, 12, 23)) return false;
    if (!status_ok(m, 23, 24, 34)) return false;
    if (!status_ok(m, 34, 35, 45)) return false;
    if (!status_ok(m, 45, 46, 56)) return false;
    memset(o, 0, sizeof(*o));
    if (mb0(m, 0, 1)) {
        int v = (int)mb0(m, 2, 11); if (mb0(m, 1, 2)) v -= 512;
        o->roll = v * 45.0f / 256.0f; o->roll_v = true;
        if (fabsf(o->roll) > 50.0f) return false;
    }
    if (mb0(m, 11, 12)) {
        int v = (int)mb0(m, 13, 23); if (mb0(m, 12, 13)) v -= 1024;
        o->trk = v * 90.0f / 512.0f; if (o->trk < 0) o->trk += 360.0f; o->trk_v = true;
    }
    if (mb0(m, 23, 24)) {
        o->gs = mb0(m, 24, 34) * 2.0f; o->gs_v = true;
        if (o->gs > 600.0f) return false;
    }
    if (mb0(m, 34, 35)) {
        int v = (int)mb0(m, 36, 45); if (mb0(m, 35, 36)) v -= 512;
        o->rate = v * 8.0f / 256.0f; o->rate_v = true;
    }
    if (mb0(m, 45, 46)) {
        o->tas = mb0(m, 46, 56) * 2.0f; o->tas_v = true;
        if (o->tas > 600.0f) return false;
    }
    if (o->gs_v && o->tas_v && fabsf(o->gs - o->tas) > 200.0f) return false;
    return o->roll_v || o->trk_v || o->gs_v || o->tas_v;
}

typedef struct {
    bool hdg_v;  float hdg;
    bool ias_v;  float ias;
    bool mach_v; float mach;
    bool vrb_v;  int32_t vrb;
    bool vri_v;  int32_t vri;
} bds60_t;

static bool is_bds60(const uint8_t *m, bds60_t *o)
{
    if (!status_ok(m, 0, 1, 12)) return false;
    if (!status_ok(m, 12, 13, 23)) return false;
    if (!status_ok(m, 23, 24, 34)) return false;
    if (!status_ok(m, 34, 35, 45)) return false;
    if (!status_ok(m, 45, 46, 56)) return false;
    memset(o, 0, sizeof(*o));
    if (mb0(m, 0, 1)) {
        int v = (int)mb0(m, 2, 12); if (mb0(m, 1, 2)) v -= 1024;
        o->hdg = v * 90.0f / 512.0f; if (o->hdg < 0) o->hdg += 360.0f; o->hdg_v = true;
    }
    if (mb0(m, 12, 13)) {
        o->ias = (float)mb0(m, 13, 23); o->ias_v = true;
        if (o->ias > 500.0f || o->ias < 30.0f) return false;
    }
    if (mb0(m, 23, 24)) {
        o->mach = mb0(m, 24, 34) * 2.048f / 512.0f; o->mach_v = true;
        if (o->mach > 1.0f) return false;
    }
    if (mb0(m, 34, 35)) {
        int v = (int)mb0(m, 36, 45); if (mb0(m, 35, 36)) v -= 512;
        o->vrb = v * 32; o->vrb_v = true;
    }
    if (mb0(m, 45, 46)) {
        int v = (int)mb0(m, 47, 56); if (mb0(m, 46, 47)) v -= 512;
        o->vri = v * 32; o->vri_v = true;
    }
    if (o->vrb_v && o->vri_v && abs(o->vrb - o->vri) > 2000) return false;
    if (o->vrb_v && abs(o->vrb) > 6000) return false;
    return o->hdg_v || o->ias_v || o->mach_v;
}

static float angle_diff(float a, float b)
{
    float d = fabsf(a - b);
    return d > 180.0f ? 360.0f - d : d;
}

void modes_decode_commb(mm_t *mm, const modes_ref_t *ref)
{
    const uint8_t *m = mm->msg;
    if (mm->len != MODES_LONG_BYTES || (mm->df != 20 && mm->df != 21)) return;
    if (mb_all_zero(m)) return;

    char cs[9];
    bool c10 = is_bds10(m);
    bool c20 = is_bds20(m, cs);
    bool c30 = is_bds30(m);
    bool c40 = is_bds40(m);
    bds50_t b50; bool c50 = is_bds50(m, &b50);
    bds60_t b60; bool c60 = is_bds60(m, &b60);

    /* Use known aircraft state to reject implausible candidates */
    if (ref) {
        if (c50 && ref->gs_valid && b50.gs_v && fabsf(b50.gs - ref->gs_kt) > 40.0f) c50 = false;
        if (c50 && ref->track_valid && b50.trk_v && angle_diff(b50.trk, ref->track_deg) > 30.0f) c50 = false;
        if (c60 && ref->gs_valid && b60.ias_v && fabsf(b60.ias - ref->gs_kt) > 250.0f) c60 = false;
        if (c60 && ref->alt_valid && b60.mach_v && b60.ias_v) {
            /* Mach and IAS must be roughly consistent at this altitude (ISA, rough) */
            float alt_m = ref->alt_ft * 0.3048f;
            float t = 288.15f - 0.0065f * fminf(alt_m, 11000.0f);
            float a_kt = 38.967854f * sqrtf(t);     /* speed of sound in kt */
            float p_ratio = powf(t / 288.15f, 5.2559f);
            float tas_est = b60.mach * a_kt;
            float ias_est = tas_est * sqrtf(p_ratio * 288.15f / t);
            if (fabsf(ias_est - b60.ias) > 80.0f) c60 = false;
        }
    }

    int n = c10 + c20 + c30 + c40 + c50 + c60;
    if (n != 1) return;

    if (c20) {
        mm->bds = 0x20;
        memcpy(mm->callsign, cs, sizeof(cs));
        mm->callsign_valid = true;
    } else if (c10) {
        mm->bds = 0x10;
    } else if (c30) {
        mm->bds = 0x30;
        mm->acas_ra_valid = true;
        mm->acas_ra_active = mb0(m, 8, 22) != 0;
    } else if (c40) {
        mm->bds = 0x40;
        if (mb0(m, 0, 1)) {
            mm->sel_alt_valid = true;
            mm->sel_alt_ft = (int32_t)mb0(m, 1, 13) * 16;
            mm->sel_alt_fms = false;
        }
        if (mb0(m, 13, 14)) {
            mm->fms_alt_valid = true;
            mm->fms_alt_ft = (int32_t)mb0(m, 14, 26) * 16;
        }
        if (mb0(m, 26, 27)) {
            mm->qnh_valid = true;
            mm->qnh_hpa = mb0(m, 27, 39) * 0.1f + 800.0f;
        }
        if (mb0(m, 47, 48)) {
            mm->nav_modes_valid = true;
            mm->vnav_on = mb0(m, 48, 49);
            mm->alt_hold_on = mb0(m, 49, 50);
            mm->approach_on = mb0(m, 50, 51);
            mm->ap_on = mm->vnav_on || mm->alt_hold_on || mm->approach_on;
            mm->lnav_on = false;
        }
    } else if (c50) {
        mm->bds = 0x50;
        if (b50.roll_v) { mm->roll_valid = true; mm->roll_deg = b50.roll; }
        if (b50.trk_v)  { mm->track_valid = true; mm->track_deg = b50.trk; }
        if (b50.gs_v)   { mm->gs_valid = true; mm->gs_kt = b50.gs; }
        if (b50.rate_v) { mm->track_rate_valid = true; mm->track_rate_dps = b50.rate; }
        if (b50.tas_v)  { mm->tas_valid = true; mm->tas_kt = b50.tas; }
    } else if (c60) {
        mm->bds = 0x60;
        if (b60.hdg_v)  { mm->heading_valid = true; mm->heading_deg = b60.hdg; mm->heading_is_true = false; }
        if (b60.ias_v)  { mm->ias_valid = true; mm->ias_kt = b60.ias; }
        if (b60.mach_v) { mm->mach_valid = true; mm->mach = b60.mach; }
        if (b60.vrb_v)  { mm->baro_rate_valid = true; mm->baro_rate_fpm = b60.vrb; }
        if (b60.vri_v)  { mm->geom_rate_valid = true; mm->geom_rate_fpm = b60.vri; }
    }
}

/* ------------------------------------------------------------------------- */
/* Top-level decode                                                          */
/* ------------------------------------------------------------------------- */

static void decode_fs(uint32_t fs, mm_t *mm)
{
    mm->fs_valid = true;
    switch (fs) {
    case 0: mm->ground_valid = true; mm->on_ground = false; break;
    case 1: mm->ground_valid = true; mm->on_ground = true; break;
    case 2: mm->ground_valid = true; mm->on_ground = false; mm->alert = true; break;
    case 3: mm->ground_valid = true; mm->on_ground = true; mm->alert = true; break;
    case 4: mm->alert = true; mm->spi = true; break;
    case 5: mm->spi = true; break;
    default: mm->fs_valid = false; break;
    }
}

bool modes_decode(const uint8_t *frame, int len_bytes, mm_t *mm,
                  bool (*known_addr)(uint32_t addr))
{
    memset(mm, 0, sizeof(*mm));
    mm->rssi_dbfs = NAN;
    if (len_bytes != MODES_SHORT_BYTES && len_bytes != MODES_LONG_BYTES) return false;
    memcpy(mm->msg, frame, len_bytes);
    mm->len = len_bytes;

    const uint8_t *m = mm->msg;
    uint8_t df = m[0] >> 3;
    if (df >= 24) return false;  /* Comm-D ELM not handled */
    mm->df = df;

    bool is_long = (df >= 16);
    if ((is_long ? MODES_LONG_BYTES : MODES_SHORT_BYTES) != len_bytes) return false;

    uint32_t resid = modes_crc_residual(m, len_bytes);

    switch (df) {
    case 11: {
        /* all-call reply: PI = CRC ^ (IC/II), accept II/SI codes only */
        if (resid & ~0x7Fu) return false;
        mm->crc_ok = true;
        mm->addr = getbits(m, 9, 32);
        mm->ca_valid = true;
        mm->ca = getbits(m, 6, 8);
        if (mm->ca == 4) { mm->ground_valid = true; mm->on_ground = true; }
        else if (mm->ca == 5) { mm->ground_valid = true; mm->on_ground = false; }
        return true;
    }
    case 17:
    case 18: {
        if (resid != 0) return false;
        mm->crc_ok = true;
        mm->addr = getbits(m, 9, 32);
        if (df == 17) {
            mm->addr_type = ADDR_ICAO;
            mm->ca_valid = true;
            mm->ca = getbits(m, 6, 8);
            if (mm->ca == 4) { mm->ground_valid = true; mm->on_ground = true; }
            else if (mm->ca == 5) { mm->ground_valid = true; mm->on_ground = false; }
        } else {
            uint8_t cf = getbits(m, 6, 8);
            switch (cf) {
            case 0: mm->addr_type = ADDR_ICAO; break;
            case 1: mm->addr_type = ADDR_ADSB_OTHER; mm->addr |= 0x1000000; break;
            case 2: mm->addr_type = ADDR_TISB_ICAO; break;
            case 6: mm->addr_type = ADDR_ADSR_ICAO; break;
            default: return false;  /* coarse TIS-B / management: different layouts */
            }
        }
        decode_es(m, mm);
        return true;
    }
    case 0: case 4: case 5: case 16: case 20: case 21: {
        /* address/parity: residual is the address */
        mm->addr = resid;
        mm->addr_from_ap = true;
        mm->addr_type = ADDR_MODE_S;
        if (!known_addr || !known_addr(mm->addr)) return false;

        if (df == 0 || df == 16) {
            mm->ground_valid = true;
            mm->on_ground = getbits(m, 6, 6);
        } else {
            decode_fs(getbits(m, 6, 8), mm);
        }
        if (df == 5 || df == 21) {
            uint32_t id = getbits(m, 20, 32);
            if (id) {
                mm->squawk_valid = true;
                mm->squawk = (uint16_t)id13_to_hexgillham(id);
            }
        } else {
            int32_t ft;
            if (decode_ac13(getbits(m, 20, 32), &ft)) {
                mm->alt_valid = true; mm->alt_ft = ft; mm->alt_src = ALT_SRC_BARO;
            }
        }
        if (df == 16 && getbits(m, 33, 40) == 0x30) {
            mm->acas_ra_valid = true;
            mm->acas_ra_active = getbits(m, 41, 54) != 0;
        }
        /* DF20/21 MB inference happens in aircraft.c with context */
        return true;
    }
    default:
        return false;
    }
}

/* ------------------------------------------------------------------------- */
/* CPR                                                                       */
/* ------------------------------------------------------------------------- */

int cpr_nl(double lat)
{
    lat = fabs(lat);
    if (lat < 1e-9) return 59;
    if (lat > 87.0) return 1;
    if (fabs(lat - 87.0) < 1e-9) return 2;
    double a = 1.0 - cos(M_PI / 30.0);
    double c = cos(M_PI / 180.0 * lat);
    double nl = floor(2.0 * M_PI / acos(1.0 - a / (c * c)));
    return (int)nl;
}

static double cpr_mod(double a, double b)
{
    double r = fmod(a, b);
    return r < 0 ? r + b : r;
}

bool cpr_decode_global_airborne(uint32_t even_lat, uint32_t even_lon,
                                uint32_t odd_lat, uint32_t odd_lon,
                                bool use_odd, double *lat, double *lon)
{
    const double scale = 131072.0;
    double latE = even_lat / scale, lonE = even_lon / scale;
    double latO = odd_lat / scale, lonO = odd_lon / scale;
    double dlatE = 360.0 / 60.0, dlatO = 360.0 / 59.0;

    double j = floor(59.0 * latE - 60.0 * latO + 0.5);
    double rlatE = dlatE * (cpr_mod(j, 60.0) + latE);
    double rlatO = dlatO * (cpr_mod(j, 59.0) + latO);
    if (rlatE >= 270.0) rlatE -= 360.0;
    if (rlatO >= 270.0) rlatO -= 360.0;
    if (rlatE < -90 || rlatE > 90 || rlatO < -90 || rlatO > 90) return false;
    if (cpr_nl(rlatE) != cpr_nl(rlatO)) return false;

    double rlat, rlon;
    if (use_odd) {
        int nl = cpr_nl(rlatO);
        int ni = nl - 1 > 1 ? nl - 1 : 1;
        double m = floor(lonE * (nl - 1) - lonO * nl + 0.5);
        rlon = (360.0 / ni) * (cpr_mod(m, ni) + lonO);
        rlat = rlatO;
    } else {
        int nl = cpr_nl(rlatE);
        int ni = nl > 1 ? nl : 1;
        double m = floor(lonE * (nl - 1) - lonO * nl + 0.5);
        rlon = (360.0 / ni) * (cpr_mod(m, ni) + lonE);
        rlat = rlatE;
    }
    rlon -= floor((rlon + 180.0) / 360.0) * 360.0;
    *lat = rlat;
    *lon = rlon;
    return true;
}

bool cpr_decode_global_surface(double ref_lat, double ref_lon,
                               uint32_t even_lat, uint32_t even_lon,
                               uint32_t odd_lat, uint32_t odd_lon,
                               bool use_odd, double *lat, double *lon)
{
    const double scale = 131072.0;
    double latE = even_lat / scale, lonE = even_lon / scale;
    double latO = odd_lat / scale, lonO = odd_lon / scale;
    double dlatE = 90.0 / 60.0, dlatO = 90.0 / 59.0;

    double j = floor(59.0 * latE - 60.0 * latO + 0.5);
    double rlatE = dlatE * (cpr_mod(j, 60.0) + latE);
    double rlatO = dlatO * (cpr_mod(j, 59.0) + latO);

    /* pick northern or southern solution nearest the reference */
    if (ref_lat < 0) { rlatE -= 90.0; rlatO -= 90.0; }
    if (cpr_nl(rlatE) != cpr_nl(rlatO)) return false;

    double rlat = use_odd ? rlatO : rlatE;
    int nl = cpr_nl(rlat);
    int ni = use_odd ? (nl - 1 > 1 ? nl - 1 : 1) : (nl > 1 ? nl : 1);
    double m = floor(lonE * (nl - 1) - lonO * nl + 0.5);
    double rlon = (90.0 / ni) * (cpr_mod(m, ni) + (use_odd ? lonO : lonE));

    /* four candidate longitudes, 90 deg apart; choose nearest reference */
    double best = rlon, best_d = 1e9;
    for (int k = 0; k < 4; k++) {
        double c = rlon + 90.0 * k;
        c -= floor((c + 180.0) / 360.0) * 360.0;
        double d = fabs(c - ref_lon);
        if (d > 180.0) d = 360.0 - d;
        if (d < best_d) { best_d = d; best = c; }
    }
    *lat = rlat;
    *lon = best;
    return true;
}

bool cpr_decode_local(double ref_lat, double ref_lon, uint32_t cpr_lat, uint32_t cpr_lon,
                      bool odd, bool surface, double *lat, double *lon)
{
    const double scale = 131072.0;
    double span = surface ? 90.0 : 360.0;
    double y = cpr_lat / scale, x = cpr_lon / scale;
    double dlat = span / (odd ? 59.0 : 60.0);

    double j = floor(ref_lat / dlat) + floor(0.5 + cpr_mod(ref_lat, dlat) / dlat - y);
    double rlat = dlat * (j + y);
    if (rlat < -90.0 || rlat > 90.0) return false;
    if (fabs(rlat - ref_lat) > dlat / 2.0) return false;

    int nl = cpr_nl(rlat) - (odd ? 1 : 0);
    double dlon = span / (nl > 1 ? nl : 1);
    double m = floor(ref_lon / dlon) + floor(0.5 + cpr_mod(ref_lon, dlon) / dlon - x);
    double rlon = dlon * (m + x);
    double dl = fabs(rlon - ref_lon);
    if (dl > 180.0) dl = 360.0 - dl;
    if (dl > dlon / 2.0) return false;
    rlon -= floor((rlon + 180.0) / 360.0) * 360.0;

    *lat = rlat;
    *lon = rlon;
    return true;
}

/* ------------------------------------------------------------------------- */
/* Strings                                                                   */
/* ------------------------------------------------------------------------- */

const char *emergency_str(emergency_t e)
{
    switch (e) {
    case EMERG_NONE:      return "None";
    case EMERG_GENERAL:   return "GENERAL EMERGENCY";
    case EMERG_LIFEGUARD: return "LIFEGUARD / MEDICAL";
    case EMERG_MINFUEL:   return "MINIMUM FUEL";
    case EMERG_NORDO:     return "NO COMMUNICATIONS";
    case EMERG_UNLAWFUL:  return "UNLAWFUL INTERFERENCE";
    case EMERG_DOWNED:    return "DOWNED AIRCRAFT";
    default:              return "Reserved";
    }
}

const char *category_str(uint8_t c)
{
    switch (c) {
    case 0xA1: return "Light (<15.5k lb)";
    case 0xA2: return "Small (15.5-75k lb)";
    case 0xA3: return "Large (75-300k lb)";
    case 0xA4: return "High vortex large (B757)";
    case 0xA5: return "Heavy (>300k lb)";
    case 0xA6: return "High performance";
    case 0xA7: return "Rotorcraft";
    case 0xB1: return "Glider / sailplane";
    case 0xB2: return "Lighter-than-air";
    case 0xB3: return "Parachutist / skydiver";
    case 0xB4: return "Ultralight / hang glider";
    case 0xB6: return "UAV / drone";
    case 0xB7: return "Space vehicle";
    case 0xC1: return "Surface: emergency vehicle";
    case 0xC2: return "Surface: service vehicle";
    case 0xC3: return "Ground obstruction";
    case 0xC4: return "Cluster obstacle";
    case 0xC5: return "Line obstacle";
    default:   return "Unspecified";
    }
}
