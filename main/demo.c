#include "demo.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "aircraft.h"
#include "board.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota.h"
#include "settings.h"
#include "ui.h"

static const char *TAG = "demo";

/* Heathrow and the points the simulated traffic uses */
#define LHR_LAT   51.4706
#define LHR_LON   -0.4619
#define THR27R_LAT 51.4775   /* east thresholds */
#define THR27R_LON -0.4331
#define THR27L_LAT 51.4647
#define THR27L_LON -0.4342

typedef enum { P_APPROACH, P_DEPART, P_HOLD, P_CRUISE, P_ORBIT, P_EMERGENCY } phase_t;

typedef struct {
    const char *icao, *iata, *city, *name;
    double lat, lon;
} apt_t;

static const apt_t LHR = {"EGLL", "LHR", "London", "London Heathrow Airport", 51.4706, -0.4619};
static const apt_t JFK = {"KJFK", "JFK", "New York", "John F. Kennedy International Airport", 40.6413, -73.7781};
static const apt_t DXB = {"OMDB", "DXB", "Dubai", "Dubai International Airport", 25.2532, 55.3657};
static const apt_t SIN = {"WSSS", "SIN", "Singapore", "Singapore Changi Airport", 1.3644, 103.9915};
static const apt_t FRA = {"EDDF", "FRA", "Frankfurt", "Frankfurt am Main Airport", 50.0379, 8.5622};
static const apt_t AMS = {"EHAM", "AMS", "Amsterdam", "Amsterdam Airport Schiphol", 52.3105, 4.7683};
static const apt_t CDG = {"LFPG", "CDG", "Paris", "Paris Charles de Gaulle Airport", 49.0097, 2.5479};
static const apt_t DUB = {"EIDW", "DUB", "Dublin", "Dublin Airport", 53.4264, -6.2499};
static const apt_t EDI = {"EGPH", "EDI", "Edinburgh", "Edinburgh Airport", 55.9500, -3.3725};
static const apt_t MAD = {"LEMD", "MAD", "Madrid", "Adolfo Suarez Madrid-Barajas Airport", 40.4983, -3.5676};
static const apt_t HND = {"RJTT", "HND", "Tokyo", "Tokyo Haneda Airport", 35.5494, 139.7798};
static const apt_t GVA = {"LSGG", "GVA", "Geneva", "Geneva Airport", 46.2381, 6.1090};
static const apt_t LGW = {"EGKK", "LGW", "London", "London Gatwick Airport", 51.1481, -0.1903};
static const apt_t BCN = {"LEBL", "BCN", "Barcelona", "Josep Tarradellas Barcelona-El Prat Airport", 41.2974, 2.0833};
static const apt_t KEF = {"BIKF", "KEF", "Reykjavik", "Keflavik International Airport", 63.9850, -22.6056};
static const apt_t ATL = {"KATL", "ATL", "Atlanta", "Hartsfield-Jackson Atlanta International Airport", 33.6407, -84.4277};
static const apt_t IST = {"LTFM", "IST", "Istanbul", "Istanbul Airport", 41.2753, 28.7519};

typedef struct {
    /* identity */
    uint32_t hex;
    const char *cs, *reg, *type, *type_name, *maker, *owner, *airline, *airline_cs;
    const apt_t *from, *to;
    uint8_t category;
    uint16_t squawk;
    phase_t phase;
    /* initial state */
    double lat0, lon0;
    float alt0, gs0, trk0;
    float cruise_alt;     /* departure/cruise target */
    float hold_r_nm;      /* orbit/hold radius */
    bool rwy_left;        /* approach: 27L instead of 27R */
    /* live state */
    double lat, lon;
    float alt, gs, trk, vr, roll, angle;
    bool active;
    int step;
} sim_t;


static sim_t s_sim[] = {
    /* arrivals on final / downwind for 27R and 27L */
    {.hex = 0x406A93, .cs = "BAW286", .reg = "G-XLEB", .type = "A388", .type_name = "Airbus A380-841", .maker = "Airbus",
     .owner = "British Airways", .airline = "British Airways", .airline_cs = "SPEEDBIRD", .from = &SIN, .to = &LHR,
     .category = 0xA5, .squawk = 0x5521, .phase = P_APPROACH, .lat0 = 51.4775, .lon0 = -0.05, .alt0 = 5200, .gs0 = 165},
    {.hex = 0x896411, .cs = "UAE1", .reg = "A6-EVB", .type = "A388", .type_name = "Airbus A380-842", .maker = "Airbus",
     .owner = "Emirates", .airline = "Emirates", .airline_cs = "EMIRATES", .from = &DXB, .to = &LHR,
     .category = 0xA5, .squawk = 0x4473, .phase = P_APPROACH, .lat0 = 51.4647, .lon0 = 0.12, .alt0 = 7000, .gs0 = 180, .rwy_left = true},
    {.hex = 0x4007F1, .cs = "VIR4", .reg = "G-VLUX", .type = "B789", .type_name = "Boeing 787-9 Dreamliner", .maker = "Boeing",
     .owner = "Virgin Atlantic Airways", .airline = "Virgin Atlantic", .airline_cs = "VIRGIN", .from = &JFK, .to = &LHR,
     .category = 0xA5, .squawk = 0x6312, .phase = P_APPROACH, .lat0 = 51.4775, .lon0 = 0.30, .alt0 = 8000, .gs0 = 200},
    {.hex = 0x3C6614, .cs = "DLH2KM", .reg = "D-AINQ", .type = "A20N", .type_name = "Airbus A320-271N", .maker = "Airbus",
     .owner = "Lufthansa", .airline = "Lufthansa", .airline_cs = "LUFTHANSA", .from = &FRA, .to = &LHR,
     .category = 0xA3, .squawk = 0x3054, .phase = P_APPROACH, .lat0 = 51.4647, .lon0 = 0.48, .alt0 = 9000, .gs0 = 210, .rwy_left = true},
    /* departures off 27R */
    {.hex = 0x4006B6, .cs = "BAW117", .reg = "G-STBH", .type = "B77W", .type_name = "Boeing 777-336ER", .maker = "Boeing",
     .owner = "British Airways", .airline = "British Airways", .airline_cs = "SPEEDBIRD", .from = &LHR, .to = &JFK,
     .category = 0xA5, .squawk = 0x2617, .phase = P_DEPART, .lat0 = 51.478, .lon0 = -0.62, .alt0 = 4000, .gs0 = 230, .trk0 = 270,
     .cruise_alt = 34000},
    {.hex = 0x4CA9E2, .cs = "EIN159", .reg = "EI-DVM", .type = "A320", .type_name = "Airbus A320-214", .maker = "Airbus",
     .owner = "Aer Lingus", .airline = "Aer Lingus", .airline_cs = "SHAMROCK", .from = &LHR, .to = &DUB,
     .category = 0xA3, .squawk = 0x2645, .phase = P_DEPART, .lat0 = 51.53, .lon0 = -0.95, .alt0 = 9500, .gs0 = 290, .trk0 = 300,
     .cruise_alt = 36000},
    {.hex = 0x405D8A, .cs = "BAW1438", .reg = "G-EUYM", .type = "A320", .type_name = "Airbus A320-232", .maker = "Airbus",
     .owner = "British Airways", .airline = "British Airways", .airline_cs = "SPEEDBIRD", .from = &LHR, .to = &EDI,
     .category = 0xA3, .squawk = 0x2671, .phase = P_DEPART, .lat0 = 51.62, .lon0 = -0.72, .alt0 = 13000, .gs0 = 320, .trk0 = 340,
     .cruise_alt = 33000},
    /* holding stacks: Bovingdon, Lambourne, Ockham, Biggin */
    {.hex = 0x7C6DB8, .cs = "QFA1", .reg = "VH-ZNE", .type = "B789", .type_name = "Boeing 787-9 Dreamliner", .maker = "Boeing",
     .owner = "Qantas", .airline = "Qantas", .airline_cs = "QANTAS", .from = &SIN, .to = &LHR,
     .category = 0xA5, .squawk = 0x4211, .phase = P_HOLD, .lat0 = 51.726, .lon0 = -0.550, .alt0 = 8000, .gs0 = 220, .hold_r_nm = 3.0f},
    {.hex = 0xA8C1F3, .cs = "AAL106", .reg = "N735AT", .type = "B77W", .type_name = "Boeing 777-323ER", .maker = "Boeing",
     .owner = "American Airlines", .airline = "American Airlines", .airline_cs = "AMERICAN", .from = &JFK, .to = &LHR,
     .category = 0xA5, .squawk = 0x1763, .phase = P_HOLD, .lat0 = 51.646, .lon0 = 0.152, .alt0 = 10000, .gs0 = 230, .hold_r_nm = 3.2f},
    {.hex = 0x89611A, .cs = "QTR11", .reg = "A7-BEC", .type = "B77W", .type_name = "Boeing 777-3DZER", .maker = "Boeing",
     .owner = "Qatar Airways", .airline = "Qatar Airways", .airline_cs = "QATARI", .from = &DXB, .to = &LHR,
     .category = 0xA5, .squawk = 0x3315, .phase = P_HOLD, .lat0 = 51.305, .lon0 = -0.447, .alt0 = 9000, .gs0 = 225, .hold_r_nm = 3.0f},
    {.hex = 0x4B1A21, .cs = "SWR318", .reg = "HB-JCN", .type = "BCS3", .type_name = "Airbus A220-300", .maker = "Airbus",
     .owner = "Swiss International Air Lines", .airline = "Swiss", .airline_cs = "SWISS", .from = &GVA, .to = &LHR,
     .category = 0xA3, .squawk = 0x5730, .phase = P_HOLD, .lat0 = 51.331, .lon0 = 0.033, .alt0 = 7000, .gs0 = 210, .hold_r_nm = 2.8f},
    /* overflights */
    {.hex = 0xA2F5C4, .cs = "DAL31", .reg = "N826NW", .type = "A333", .type_name = "Airbus A330-323", .maker = "Airbus",
     .owner = "Delta Air Lines", .airline = "Delta Air Lines", .airline_cs = "DELTA", .from = &AMS, .to = &ATL,
     .category = 0xA5, .squawk = 0x1407, .phase = P_CRUISE, .lat0 = 51.95, .lon0 = 0.55, .alt0 = 37000, .gs0 = 468, .trk0 = 262},
    {.hex = 0x4BB869, .cs = "THY1979", .reg = "TC-LJB", .type = "B38M", .type_name = "Boeing 737 MAX 8", .maker = "Boeing",
     .owner = "Turkish Airlines", .airline = "Turkish Airlines", .airline_cs = "TURKISH", .from = &IST, .to = &DUB,
     .category = 0xA3, .squawk = 0x7146, .phase = P_CRUISE, .lat0 = 51.20, .lon0 = 0.70, .alt0 = 39000, .gs0 = 455, .trk0 = 292},
    {.hex = 0x86E4A2, .cs = "JAL44", .reg = "JA861J", .type = "B789", .type_name = "Boeing 787-9 Dreamliner", .maker = "Boeing",
     .owner = "Japan Airlines", .airline = "Japan Airlines", .airline_cs = "JAPANAIR", .from = &HND, .to = &LHR,
     .category = 0xA5, .squawk = 0x2272, .phase = P_CRUISE, .lat0 = 52.15, .lon0 = 0.10, .alt0 = 25000, .gs0 = 410, .trk0 = 225},
    {.hex = 0x392AF3, .cs = "AFR1680", .reg = "F-HEPA", .type = "A20N", .type_name = "Airbus A320-271N", .maker = "Airbus",
     .owner = "Air France", .airline = "Air France", .airline_cs = "AIRFRANS", .from = &CDG, .to = &DUB,
     .category = 0xA3, .squawk = 0x6630, .phase = P_CRUISE, .lat0 = 50.95, .lon0 = -0.30, .alt0 = 35000, .gs0 = 440, .trk0 = 318},
    {.hex = 0x484F6D, .cs = "KLM1015", .reg = "PH-EXG", .type = "E295", .type_name = "Embraer E195-E2", .maker = "Embraer",
     .owner = "KLM Cityhopper", .airline = "KLM", .airline_cs = "KLM", .from = &AMS, .to = &LHR,
     .category = 0xA3, .squawk = 0x5142, .phase = P_CRUISE, .lat0 = 51.75, .lon0 = 0.75, .alt0 = 17000, .gs0 = 330, .trk0 = 245},
    {.hex = 0x4CA4E8, .cs = "RYR8YK", .reg = "EI-IHB", .type = "B38M", .type_name = "Boeing 737 MAX 8-200", .maker = "Boeing",
     .owner = "Ryanair", .airline = "Ryanair", .airline_cs = "RYANAIR", .from = &BCN, .to = &DUB,
     .category = 0xA3, .squawk = 0x0563, .phase = P_CRUISE, .lat0 = 50.85, .lon0 = 0.15, .alt0 = 38000, .gs0 = 452, .trk0 = 330},
    {.hex = 0x4CC56A, .cs = "ICE450", .reg = "TF-ICA", .type = "B39M", .type_name = "Boeing 737 MAX 9", .maker = "Boeing",
     .owner = "Icelandair", .airline = "Icelandair", .airline_cs = "ICEAIR", .from = &KEF, .to = &LHR,
     .category = 0xA3, .squawk = 0x2510, .phase = P_CRUISE, .lat0 = 52.25, .lon0 = -1.05, .alt0 = 21000, .gs0 = 380, .trk0 = 140},
    {.hex = 0x406D21, .cs = "EZY63CR", .reg = "G-UZHK", .type = "A20N", .type_name = "Airbus A320-251N", .maker = "Airbus",
     .owner = "easyJet", .airline = "easyJet", .airline_cs = "EASY", .from = &LGW, .to = &MAD,
     .category = 0xA3, .squawk = 0x4632, .phase = P_CRUISE, .lat0 = 51.10, .lon0 = -0.35, .alt0 = 12000, .gs0 = 300, .trk0 = 205},
    /* general aviation, police helicopter, military */
    {.hex = 0x405A37, .cs = "GBNHK", .reg = "G-BNHK", .type = "C172", .type_name = "Cessna 172N Skyhawk", .maker = "Cessna",
     .owner = "Private", .category = 0xA1, .squawk = 0x7000, .phase = P_CRUISE,
     .lat0 = 51.69, .lon0 = -0.27, .alt0 = 2400, .gs0 = 95, .trk0 = 105},
    {.hex = 0x407A1E, .cs = "UKP3", .reg = "G-POLB", .type = "EC35", .type_name = "Airbus Helicopters EC135 P2+", .maker = "Airbus Helicopters",
     .owner = "National Police Air Service", .category = 0xA7, .squawk = 0x0032, .phase = P_ORBIT,
     .lat0 = 51.507, .lon0 = -0.127, .alt0 = 1500, .gs0 = 85, .hold_r_nm = 1.1f},
    {.hex = 0x43C6E4, .cs = "RRR4271", .reg = "ZM409", .type = "A400", .type_name = "Airbus A400M Atlas C1", .maker = "Airbus Defence and Space",
     .owner = "Royal Air Force", .category = 0xA5, .squawk = 0x3721, .phase = P_CRUISE,
     .lat0 = 51.65, .lon0 = -1.35, .alt0 = 21000, .gs0 = 330, .trk0 = 75},
    /* only when ?emerg=1: a diversion squawking 7700 */
    {.hex = 0x3C70A5, .cs = "EWG7GA", .reg = "D-AEWK", .type = "A320", .type_name = "Airbus A320-214", .maker = "Airbus",
     .owner = "Eurowings", .airline = "Eurowings", .airline_cs = "EUROWINGS", .from = &FRA, .to = &DUB,
     .category = 0xA3, .squawk = 0x7700, .phase = P_EMERGENCY, .lat0 = 51.30, .lon0 = 0.55, .alt0 = 14000, .gs0 = 290, .trk0 = 285},
};
#define NSIM (int)(sizeof(s_sim) / sizeof(s_sim[0]))

static volatile bool s_emerg;

/* ------------------------------------------------------------------------- */
/* Physics                                                                   */
/* ------------------------------------------------------------------------- */

static double rad(double d) { return d * M_PI / 180.0; }

static void move(double *lat, double *lon, float trk, float nm)
{
    *lat += nm * cos(rad(trk)) / 60.0;
    *lon += nm * sin(rad(trk)) / 60.0 / cos(rad(*lat));
}

static float bearing_to(double lat1, double lon1, double lat2, double lon2)
{
    double y = sin(rad(lon2 - lon1)) * cos(rad(lat2));
    double x = cos(rad(lat1)) * sin(rad(lat2)) - sin(rad(lat1)) * cos(rad(lat2)) * cos(rad(lon2 - lon1));
    double b = atan2(y, x) * 180.0 / M_PI;
    return (float)(b < 0 ? b + 360.0 : b);
}

static float dist_nm(double lat1, double lon1, double lat2, double lon2)
{
    double dy = (lat2 - lat1) * 60.0, dx = (lon2 - lon1) * 60.0 * cos(rad(lat1));
    return (float)sqrt(dx * dx + dy * dy);
}

static float turn_towards(float cur, float want, float max_step)
{
    float d = fmodf(want - cur + 540.0f, 360.0f) - 180.0f;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    return fmodf(cur + d + 360.0f, 360.0f);
}

static void reset(sim_t *s)
{
    s->lat = s->lat0;
    s->lon = s->lon0;
    s->alt = s->alt0;
    s->gs = s->gs0;
    s->trk = s->trk0;
    s->vr = 0;
    s->roll = 0;
    s->angle = 0;
    s->step = 0;
    if (s->phase == P_APPROACH) s->trk = 270;
}

/* advance one aircraft by dt seconds; returns false when it should respawn */
static bool step(sim_t *s, float dt)
{
    float nm = s->gs * dt / 3600.0f;
    s->step++;
    switch (s->phase) {
    case P_APPROACH: {
        double tl = s->rwy_left ? THR27L_LAT : THR27R_LAT, tn = s->rwy_left ? THR27L_LON : THR27R_LON;
        float d = dist_nm(s->lat, s->lon, tl, tn);
        s->trk = turn_towards(s->trk, bearing_to(s->lat, s->lon, tl, tn), 3.0f * dt);
        float want_alt = d * 318.0f;          /* 3 degree glidepath */
        if (want_alt < s->alt) s->vr = -fminf(1400.0f, (s->alt - want_alt) * 60.0f / fmaxf(dt, 1.0f));
        else s->vr = 0;
        s->alt = fmaxf(0, s->alt + s->vr * dt / 60.0f);
        s->gs = fmaxf(140.0f, s->gs - 0.6f * dt);
        s->roll = 0;
        move(&s->lat, &s->lon, s->trk, nm);
        return d > 0.4f;
    }
    case P_DEPART: {
        float want = s->step * dt < 60 ? s->trk0 : bearing_to(s->lat, s->lon, s->to->lat, s->to->lon);
        float before = s->trk;
        s->trk = turn_towards(s->trk, want, 2.5f * dt);
        s->roll = fabsf(s->trk - before) > 0.1f ? 20.0f : 0.0f;
        if (s->alt < s->cruise_alt) { s->vr = s->alt < 10000 ? 2600 : 1800; s->alt += s->vr * dt / 60.0f; }
        else { s->vr = 0; s->alt = s->cruise_alt; }
        s->gs = fminf(s->alt < 10000 ? 290.0f : 470.0f, s->gs + 0.8f * dt);
        move(&s->lat, &s->lon, s->trk, nm);
        return dist_nm(s->lat, s->lon, LHR_LAT, LHR_LON) < 75.0f;
    }
    case P_HOLD:
    case P_ORBIT: {
        /* constant-radius orbit around the fix */
        float r = s->hold_r_nm;
        s->angle += (nm / r) * 180.0f / (float)M_PI;
        s->lat = s->lat0 + r * cos(rad(s->angle)) / 60.0;
        s->lon = s->lon0 + r * sin(rad(s->angle)) / 60.0 / cos(rad(s->lat0));
        s->trk = fmodf(s->angle + 90.0f, 360.0f);
        s->roll = s->phase == P_ORBIT ? 20.0f : 25.0f;
        s->vr = 0;
        return true;
    }
    case P_EMERGENCY:
        /* descending towards Heathrow */
        s->trk = turn_towards(s->trk, bearing_to(s->lat, s->lon, LHR_LAT, LHR_LON), 1.5f * dt);
        s->vr = -2200;
        s->alt = fmaxf(4000, s->alt + s->vr * dt / 60.0f);
        move(&s->lat, &s->lon, s->trk, nm);
        return dist_nm(s->lat, s->lon, LHR_LAT, LHR_LON) > 6.0f;
    case P_CRUISE:
    default:
        s->vr = 0;
        s->roll = 0;
        move(&s->lat, &s->lon, s->trk, nm);
        return dist_nm(s->lat, s->lon, LHR_LAT, LHR_LON) < 85.0f;
    }
}

/* ------------------------------------------------------------------------- */
/* Feeding the tracker                                                       */
/* ------------------------------------------------------------------------- */

static void enrich(const sim_t *s)
{
    aircraft_lock();
    aircraft_t *a = aircraft_find(s->hex);
    if (a) {
        strlcpy(a->registration, s->reg, sizeof(a->registration));
        strlcpy(a->type_code, s->type, sizeof(a->type_code));
        strlcpy(a->type_name, s->type_name, sizeof(a->type_name));
        strlcpy(a->manufacturer, s->maker, sizeof(a->manufacturer));
        strlcpy(a->owner, s->owner, sizeof(a->owner));
        strlcpy(a->owner_country, a->country ? a->country : "", sizeof(a->owner_country));
        strlcpy(a->airline, s->airline ? s->airline : "", sizeof(a->airline));
        strlcpy(a->airline_callsign, s->airline_cs ? s->airline_cs : "", sizeof(a->airline_callsign));
        strlcpy(a->lookup_callsign, s->cs, sizeof(a->lookup_callsign));
        if (s->from && s->to) {
            const apt_t *ap[2] = {s->from, s->to};
            airport_t *dst[2] = {&a->origin, &a->destination};
            for (int i = 0; i < 2; i++) {
                memset(dst[i], 0, sizeof(airport_t));
                strlcpy(dst[i]->icao, ap[i]->icao, sizeof(dst[i]->icao));
                strlcpy(dst[i]->iata, ap[i]->iata, sizeof(dst[i]->iata));
                strlcpy(dst[i]->city, ap[i]->city, sizeof(dst[i]->city));
                strlcpy(dst[i]->name, ap[i]->name, sizeof(dst[i]->name));
                dst[i]->lat = ap[i]->lat;
                dst[i]->lon = ap[i]->lon;
                dst[i]->valid = true;
            }
        }
        a->lookup = LOOKUP_DONE;
        a->lookup_ms = now_ms();
    }
    aircraft_unlock();
}

static void report(sim_t *s, int seq)
{
    mm_t m;
    memset(&m, 0, sizeof(m));
    m.addr = s->hex;
    m.crc_ok = true;
    m.df = 17;
    float dist = dist_nm(s->lat, s->lon, LHR_LAT, LHR_LON);
    m.rssi_dbfs = -4.0f - dist * 0.32f - (float)(seq % 5);

    strlcpy(m.callsign, s->cs, sizeof(m.callsign));
    m.callsign_valid = true;
    m.category = s->category;
    m.category_valid = true;
    m.squawk = s->squawk;
    m.squawk_valid = true;
    m.ground_valid = true;
    m.on_ground = false;
    m.alt_valid = true;
    m.alt_ft = (int32_t)(s->alt / 25) * 25;
    m.alt_src = ALT_SRC_BARO;
    m.gnss_baro_diff_valid = true;
    m.gnss_baro_diff_ft = 175;
    m.gs_valid = true;
    m.gs_kt = s->gs;
    m.track_valid = true;
    m.track_deg = s->trk;
    m.baro_rate_valid = true;
    m.baro_rate_fpm = (int32_t)(s->vr / 64) * 64;
    m.geom_rate_valid = true;
    m.geom_rate_fpm = m.baro_rate_fpm + 64;

    /* air data consistent with the ISA atmosphere and a light westerly wind */
    float alt = s->alt;
    float t_k = 288.15f - 0.0019812f * fminf(alt, 36089.0f);
    float sos = 38.967854f * sqrtf(t_k);
    float wind = 12.0f + alt / 2000.0f;
    float tas = s->gs + wind * cosf((float)rad(s->trk - 270.0f));
    float sigma = powf(t_k / 288.15f, 4.2559f);
    m.tas_valid = true;
    m.tas_kt = tas;
    m.ias_valid = true;
    m.ias_kt = tas * sqrtf(sigma);
    m.mach_valid = alt > 10000;
    m.mach = tas / sos;
    m.heading_valid = true;
    m.heading_deg = fmodf(s->trk - 3.0f + 360.0f, 360.0f);
    m.roll_valid = true;
    m.roll_deg = s->roll;
    m.track_rate_valid = true;
    m.track_rate_dps = s->roll > 1 ? 1.8f : 0.0f;

    /* autopilot / intent */
    m.sel_alt_valid = true;
    m.sel_alt_ft = s->phase == P_DEPART ? (int32_t)s->cruise_alt :
                   s->phase == P_APPROACH ? 2000 :
                   s->phase == P_EMERGENCY ? 4000 : (int32_t)s->alt0;
    m.qnh_valid = true;
    m.qnh_hpa = alt < 6000 ? 1018.0f : 1013.2f;
    m.sel_hdg_valid = true;
    m.sel_hdg_deg = m.heading_deg;
    m.nav_modes_valid = s->category != 0xA1 && s->category != 0xA7;
    m.ap_on = true;
    m.lnav_on = s->phase != P_HOLD;
    m.vnav_on = s->phase == P_DEPART || s->phase == P_CRUISE;
    m.alt_hold_on = s->phase == P_HOLD || (s->phase == P_CRUISE && s->vr == 0);
    m.approach_on = s->phase == P_APPROACH && dist < 12;
    m.tcas_on = true;

    /* quality / status */
    m.opstatus_valid = true;
    m.adsb_version = 2;
    m.nac_p = 10; m.nac_p_valid = true;
    m.nac_v = 2;  m.nac_v_valid = true;
    m.sil = 3;    m.sil_valid = true;
    m.sda = 2;    m.sda_valid = true;
    m.gva = 2;    m.gva_valid = true;
    m.nic_baro = 1; m.nic_baro_valid = true;
    m.acas_op_valid = true;
    m.acas_operational = s->category != 0xA1;
    m.cap_valid = true;
    m.es_in = true;
    m.hrd_valid = true;
    m.heading_is_true = false;
    static const uint8_t bds[] = {0x20, 0x40, 0x50, 0x60};
    m.bds = bds[seq % 4];
    if (s->phase == P_EMERGENCY) {
        m.emergency_valid = true;
        m.emergency = EMERG_GENERAL;
    }
    aircraft_ingest_decoded(&m, SRC_ADSB, true, s->lat, s->lon);
}

static void sim_task(void *arg)
{
    /* build ~15 minutes of history quickly so the radar starts with trails
     * (4 s steps keep each hop within the tracker's plausibility check) */
    for (int i = 0; i < NSIM; i++) {
        reset(&s_sim[i]);
        s_sim[i].active = s_sim[i].phase != P_EMERGENCY;
    }
    for (int k = 0; k < 225; k++) {
        for (int i = 0; i < NSIM; i++) {
            sim_t *s = &s_sim[i];
            if (!s->active) continue;
            if (!step(s, 4.0f)) { aircraft_remove(s->hex); reset(s); }
            report(s, k);
        }
        if (k % 25 == 0) vTaskDelay(1);
    }
    for (int i = 0; i < NSIM; i++) if (s_sim[i].active) enrich(&s_sim[i]);
    ESP_LOGI(TAG, "simulating %d aircraft around EGLL", NSIM - 1);

    int seq = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        seq++;
        int n = 0;
        for (int i = 0; i < NSIM; i++) {
            sim_t *s = &s_sim[i];
            bool want = s->phase != P_EMERGENCY || s_emerg;
            if (!want) {
                if (s->active) { aircraft_remove(s->hex); s->active = false; }
                continue;
            }
            if (!s->active) { reset(s); s->active = true; }
            if (!step(s, 1.0f)) { aircraft_remove(s->hex); reset(s); }
            report(s, seq + i);
            enrich(s);
            n++;
        }
        aircraft_count_messages(n * 9, 0);
    }
}

/* ------------------------------------------------------------------------- */
/* Screenshots                                                               */
/* ------------------------------------------------------------------------- */

/* GET /screenshot.bmp[?tab=0..3][&sel=HEX|none][&range=NM][&emerg=0|1]
 * Applies the requested view, waits for it to render, returns a 24-bit BMP. */
static esp_err_t screenshot_get(httpd_req_t *req)
{
    char q[128] = "", v[16];
    int tab = -1, range = 0;
    int64_t sel = -1;
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "tab", v, sizeof(v)) == ESP_OK) tab = atoi(v);
        if (httpd_query_key_value(q, "range", v, sizeof(v)) == ESP_OK) range = atoi(v);
        if (httpd_query_key_value(q, "sel", v, sizeof(v)) == ESP_OK) sel = strcmp(v, "none") ? (int64_t)strtoul(v, NULL, 16) : 0;
        if (httpd_query_key_value(q, "emerg", v, sizeof(v)) == ESP_OK) s_emerg = atoi(v) != 0;
    }
    ui_demo_request(tab, sel, range);
    vTaskDelay(pdMS_TO_TICKS(3000));   /* a couple of UI refreshes */

    const int w = BOARD_LCD_H_RES, h = BOARD_LCD_V_RES;
    const uint16_t *fb = board_front_buffer();
    if (!fb) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no frame yet");
    uint16_t *copy = heap_caps_malloc(w * h * 2, MALLOC_CAP_SPIRAM);
    if (!copy) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    memcpy(copy, fb, w * h * 2);

    const uint32_t row = w * 3, img = row * h, file = 54 + img;
    uint8_t hdr[54] = {'B', 'M'};
    memcpy(hdr + 2, &file, 4);
    uint32_t off = 54, dib = 40, planes_bpp = 1 | (24 << 16), zero = 0;
    int32_t iw = w, ih = h;
    memcpy(hdr + 10, &off, 4);
    memcpy(hdr + 14, &dib, 4);
    memcpy(hdr + 18, &iw, 4);
    memcpy(hdr + 22, &ih, 4);
    memcpy(hdr + 26, &planes_bpp, 4);
    memcpy(hdr + 30, &zero, 4);
    memcpy(hdr + 34, &img, 4);
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_send_chunk(req, (const char *)hdr, sizeof(hdr));

    const int rows_per_chunk = 16;
    uint8_t *buf = malloc(row * rows_per_chunk);
    for (int y0 = h - 1; buf && y0 >= 0; y0 -= rows_per_chunk) {
        int n = 0;
        for (int y = y0; y > y0 - rows_per_chunk && y >= 0; y--, n++) {
            const uint16_t *src = copy + y * w;
            uint8_t *d = buf + n * row;
            for (int x = 0; x < w; x++) {
                uint16_t p = src[x];
                d[3 * x + 0] = (uint8_t)((p & 0x1F) << 3 | (p & 0x1F) >> 2);
                d[3 * x + 1] = (uint8_t)(((p >> 5) & 0x3F) << 2 | ((p >> 5) & 0x3F) >> 4);
                d[3 * x + 2] = (uint8_t)((p >> 11) << 3 | (p >> 11) >> 2);
            }
        }
        if (httpd_resp_send_chunk(req, (const char *)buf, n * row) != ESP_OK) break;
    }
    free(buf);
    free(copy);
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* ------------------------------------------------------------------------- */

void demo_prepare_settings(void)
{
    app_settings_t *c = settings_get();
    c->rx_lat = LHR_LAT;
    c->rx_lon = LHR_LON;
    c->rx_pos_valid = true;
    c->range_nm = 40;
    c->units = UNITS_AVIATION;
    c->tz = 9;                 /* UK / Ireland */
    c->online_lookup = true;   /* the simulator fills in the lookup results itself */
    c->saver = false;
    settings_apply_tz();
}

void demo_start(void)
{
    static const httpd_uri_t uri = {.uri = "/screenshot.bmp", .method = HTTP_GET, .handler = screenshot_get};
    ota_register_uri(&uri);
    xTaskCreatePinnedToCore(sim_task, "demo", 6144, NULL, 4, NULL, 1);
    ESP_LOGW(TAG, "DEMO MODE: simulated traffic, screenshots at /screenshot.bmp");
}
