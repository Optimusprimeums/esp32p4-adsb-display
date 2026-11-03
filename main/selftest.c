/*
 * Boot-time decoder self-test against published reference frames
 * ("The 1090MHz Riddle" and the pyModeS test suite). Results go to the log.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "modes.h"

static const char *TAG = "selftest";
static int s_fail;

static void hex2bin(const char *h, uint8_t *out, int *len)
{
    int n = strlen(h) / 2;
    for (int i = 0; i < n; i++) {
        char b[3] = {h[2 * i], h[2 * i + 1], 0};
        out[i] = (uint8_t)strtoul(b, NULL, 16);
    }
    *len = n;
}

static bool any_addr(uint32_t a) { return true; }

static bool decode(const char *hex, mm_t *mm)
{
    uint8_t f[14];
    int n;
    hex2bin(hex, f, &n);
    return modes_decode(f, n, mm, any_addr);
}

#define CHECK(cond, ...) do { \
    if (!(cond)) { s_fail++; ESP_LOGE(TAG, "FAIL: " __VA_ARGS__); } } while (0)
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

int modes_selftest(void)
{
    mm_t m, e, o;
    s_fail = 0;

    /* identification */
    CHECK(decode("8D4840D6202CC371C32CE0576098", &m), "ident decode");
    CHECK(m.addr == 0x4840D6, "ident addr %06lX", (unsigned long)m.addr);
    CHECK(m.callsign_valid && !strcmp(m.callsign, "KLM1023"), "callsign '%s'", m.callsign);

    /* airborne position pair */
    CHECK(decode("8D40621D58C382D690C8AC2863A7", &e), "even pos decode");
    CHECK(decode("8D40621D58C386435CC412692AD6", &o), "odd pos decode");
    CHECK(e.alt_valid && e.alt_ft == 38000, "altitude %ld", (long)e.alt_ft);
    CHECK(e.cpr_valid && !e.cpr_odd && o.cpr_valid && o.cpr_odd, "cpr flags");
    double lat, lon;
    bool ok = cpr_decode_global_airborne(e.cpr_lat, e.cpr_lon, o.cpr_lat, o.cpr_lon, false, &lat, &lon);
    CHECK(ok && NEAR(lat, 52.2572, 0.001) && NEAR(lon, 3.9194, 0.001), "global CPR %.5f %.5f", lat, lon);
    ok = cpr_decode_local(52.258, 3.918, e.cpr_lat, e.cpr_lon, false, false, &lat, &lon);
    CHECK(ok && NEAR(lat, 52.2572, 0.001) && NEAR(lon, 3.9194, 0.001), "local CPR %.5f %.5f", lat, lon);

    /* ground speed velocity */
    CHECK(decode("8D485020994409940838175B284F", &m), "velocity decode");
    CHECK(m.gs_valid && NEAR(m.gs_kt, 159.2, 0.5), "gs %.1f", m.gs_kt);
    CHECK(m.track_valid && NEAR(m.track_deg, 182.88, 0.1), "track %.2f", m.track_deg);
    CHECK((m.baro_rate_valid && m.baro_rate_fpm == -832) || (m.geom_rate_valid && m.geom_rate_fpm == -832), "vrate");

    /* airspeed velocity */
    CHECK(decode("8DA05F219B06B6AF189400CBC33F", &m), "airspeed decode");
    CHECK(m.heading_valid && NEAR(m.heading_deg, 243.98, 0.1), "heading %.2f", m.heading_deg);
    CHECK(m.tas_valid && NEAR(m.tas_kt, 375, 0.5), "tas %.1f", m.tas_kt);
    CHECK((m.baro_rate_valid && m.baro_rate_fpm == -2304) || (m.geom_rate_valid && m.geom_rate_fpm == -2304), "vrate2");

    /* Comm-B */
    CHECK(decode("A000083E202CC371C31DE0AA1CCF", &m), "bds20 decode");
    modes_decode_commb(&m, NULL);
    CHECK(m.bds == 0x20 && !strcmp(m.callsign, "KLM1017"), "bds20 '%s' bds=%02X", m.callsign, m.bds);

    CHECK(decode("A000029C85E42F313000007047D3", &m), "bds40 decode");
    modes_decode_commb(&m, NULL);
    CHECK(m.bds == 0x40 && m.sel_alt_ft == 3008 && NEAR(m.qnh_hpa, 1020.0, 0.05), "bds40 alt %ld qnh %.1f bds=%02X",
          (long)m.sel_alt_ft, m.qnh_hpa, m.bds);

    CHECK(decode("A000139381951536E024D4CCF6B5", &m), "bds50 decode");
    modes_decode_commb(&m, NULL);
    CHECK(m.bds == 0x50 && NEAR(m.roll_deg, 2.1, 0.05) && NEAR(m.gs_kt, 438, 0.5) && NEAR(m.tas_kt, 424, 0.5),
          "bds50 roll %.2f gs %.0f tas %.0f bds=%02X", m.roll_deg, m.gs_kt, m.tas_kt, m.bds);

    CHECK(decode("A00004128F39F91A7E27C46ADC21", &m), "bds60 decode");
    modes_decode_commb(&m, NULL);
    CHECK(m.bds == 0x60 && NEAR(m.heading_deg, 42.71, 0.05) && NEAR(m.ias_kt, 252, 0.5) &&
          NEAR(m.mach, 0.42, 0.005) && m.baro_rate_fpm == -1920,
          "bds60 hdg %.2f ias %.0f mach %.3f vr %ld bds=%02X", m.heading_deg, m.ias_kt, m.mach,
          (long)m.baro_rate_fpm, m.bds);

    /* corrupted frame must be rejected */
    CHECK(!decode("8D4840D6202CC371C32CE0576099", &m), "bad CRC accepted");

    if (s_fail) ESP_LOGE(TAG, "decoder self-test: %d FAILED", s_fail);
    else ESP_LOGI(TAG, "decoder self-test: all passed");
    return s_fail;
}
