#include "lookup.h"

#include <ctype.h>
#include <stdbool.h>
#include <string.h>
#include "aircraft.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "settings.h"
#include "wifi.h"

static const char *TAG = "lookup";

#define API_BASE          "https://api.adsbdb.com/v0"
#define JSON_MAX          (12 * 1024)
#define PHOTO_MAX         (160 * 1024)
#define AC_CACHE_N        512
#define ROUTE_CACHE_N     256
#define REQ_INTERVAL_MS   1200
#define ERROR_BACKOFF_MS  30000

typedef struct {
    uint32_t addr;
    bool used, found;
    char registration[16];
    char type_code[8];
    char type_name[48];
    char manufacturer[40];
    char owner[64];
    char owner_country[40];
    char photo_url[192];
} ac_cache_t;

typedef struct {
    char callsign[9];
    bool used, found;
    char airline[48];
    char airline_callsign[24];
    airport_t origin, destination;
} route_cache_t;

static ac_cache_t *s_ac_cache;
static route_cache_t *s_route_cache;
static int s_ac_next, s_route_next;
static volatile uint32_t s_priority;
static uint32_t s_photo_addr_done;

static SemaphoreHandle_t s_photo_lock;
static uint8_t *s_photo_buf;
static size_t s_photo_len;
static uint32_t s_photo_addr;

static lookup_stats_t s_stats;

void lookup_set_priority(uint32_t addr) { s_priority = addr; }
void lookup_get_stats(lookup_stats_t *out) { *out = s_stats; }

uint8_t *lookup_photo_take(uint32_t *addr, size_t *len)
{
    if (!s_photo_lock) return NULL;
    xSemaphoreTake(s_photo_lock, portMAX_DELAY);
    uint8_t *p = s_photo_buf;
    if (p) {
        *addr = s_photo_addr;
        *len = s_photo_len;
        s_photo_buf = NULL;
    }
    xSemaphoreGive(s_photo_lock);
    return p;
}

/* ------------------------------------------------------------------------- */
/* HTTP                                                                      */
/* ------------------------------------------------------------------------- */

/* GET url into a PSRAM buffer. Returns HTTP status, -1 on transport error. */
static int http_get(const char *url, uint8_t **out, int *out_len, int max_len)
{
    *out = NULL;
    *out_len = 0;
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 8000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 2048,
        .user_agent = "esp32p4-adsb-display/1.0",
        .max_redirection_count = 3,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return -1;
    int status = -1;
    if (esp_http_client_open(c, 0) == ESP_OK) {
        esp_http_client_fetch_headers(c);
        status = esp_http_client_get_status_code(c);
        uint8_t *buf = heap_caps_malloc(max_len + 1, MALLOC_CAP_SPIRAM);
        int total = 0;
        if (buf) {
            while (total < max_len) {
                int r = esp_http_client_read(c, (char *)buf + total, max_len - total);
                if (r <= 0) break;
                total += r;
            }
            buf[total] = 0;
            *out = buf;
            *out_len = total;
        }
    } else {
        strlcpy(s_stats.last_error, "connect failed", sizeof(s_stats.last_error));
    }
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    return status;
}

static void jstr(const cJSON *o, const char *key, char *dst, size_t n)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, key);
    if (cJSON_IsString(v) && v->valuestring) strlcpy(dst, v->valuestring, n);
    else dst[0] = 0;
}

static void parse_airport(const cJSON *o, airport_t *ap)
{
    memset(ap, 0, sizeof(*ap));
    if (!cJSON_IsObject(o)) return;
    jstr(o, "name", ap->name, sizeof(ap->name));
    jstr(o, "icao_code", ap->icao, sizeof(ap->icao));
    jstr(o, "iata_code", ap->iata, sizeof(ap->iata));
    jstr(o, "municipality", ap->city, sizeof(ap->city));
    jstr(o, "country_name", ap->country, sizeof(ap->country));
    const cJSON *lat = cJSON_GetObjectItemCaseSensitive(o, "latitude");
    const cJSON *lon = cJSON_GetObjectItemCaseSensitive(o, "longitude");
    if (cJSON_IsNumber(lat) && cJSON_IsNumber(lon)) {
        ap->lat = lat->valuedouble;
        ap->lon = lon->valuedouble;
    }
    ap->valid = ap->icao[0] || ap->name[0];
}

static void parse_route(const cJSON *fr, route_cache_t *rc)
{
    const cJSON *al = cJSON_GetObjectItemCaseSensitive(fr, "airline");
    if (cJSON_IsObject(al)) {
        jstr(al, "name", rc->airline, sizeof(rc->airline));
        jstr(al, "callsign", rc->airline_callsign, sizeof(rc->airline_callsign));
    }
    parse_airport(cJSON_GetObjectItemCaseSensitive(fr, "origin"), &rc->origin);
    parse_airport(cJSON_GetObjectItemCaseSensitive(fr, "destination"), &rc->destination);
    rc->found = rc->origin.valid || rc->destination.valid || rc->airline[0];
}

/* ------------------------------------------------------------------------- */
/* Cache                                                                     */
/* ------------------------------------------------------------------------- */

static ac_cache_t *ac_cache_find(uint32_t addr)
{
    for (int i = 0; i < AC_CACHE_N; i++) {
        if (s_ac_cache[i].used && s_ac_cache[i].addr == addr) return &s_ac_cache[i];
    }
    return NULL;
}

static ac_cache_t *ac_cache_new(uint32_t addr)
{
    ac_cache_t *e = &s_ac_cache[s_ac_next];
    s_ac_next = (s_ac_next + 1) % AC_CACHE_N;
    memset(e, 0, sizeof(*e));
    e->used = true;
    e->addr = addr;
    return e;
}

static route_cache_t *route_cache_find(const char *cs)
{
    if (!cs[0]) return NULL;
    for (int i = 0; i < ROUTE_CACHE_N; i++) {
        if (s_route_cache[i].used && strcmp(s_route_cache[i].callsign, cs) == 0) return &s_route_cache[i];
    }
    return NULL;
}

static route_cache_t *route_cache_new(const char *cs)
{
    route_cache_t *e = &s_route_cache[s_route_next];
    s_route_next = (s_route_next + 1) % ROUTE_CACHE_N;
    memset(e, 0, sizeof(*e));
    e->used = true;
    strlcpy(e->callsign, cs, sizeof(e->callsign));
    return e;
}

/* Copy cached results onto the live aircraft (takes the aircraft lock) */
static void publish(uint32_t addr, const ac_cache_t *ac, const route_cache_t *rt, lookup_state_t st)
{
    aircraft_lock();
    aircraft_t *a = aircraft_find(addr);
    if (a) {
        if (ac && ac->found) {
            strlcpy(a->registration, ac->registration, sizeof(a->registration));
            strlcpy(a->type_code, ac->type_code, sizeof(a->type_code));
            strlcpy(a->type_name, ac->type_name, sizeof(a->type_name));
            strlcpy(a->manufacturer, ac->manufacturer, sizeof(a->manufacturer));
            strlcpy(a->owner, ac->owner, sizeof(a->owner));
            strlcpy(a->owner_country, ac->owner_country, sizeof(a->owner_country));
        }
        if (rt) {
            strlcpy(a->lookup_callsign, rt->callsign, sizeof(a->lookup_callsign));
            if (rt->found) {
                strlcpy(a->airline, rt->airline, sizeof(a->airline));
                strlcpy(a->airline_callsign, rt->airline_callsign, sizeof(a->airline_callsign));
                a->origin = rt->origin;
                a->destination = rt->destination;
            } else {
                a->airline[0] = 0;
                a->origin.valid = a->destination.valid = false;
            }
        }
        a->lookup = st;
        a->lookup_ms = now_ms();
    }
    aircraft_unlock();
}

/* ------------------------------------------------------------------------- */
/* Worker                                                                    */
/* ------------------------------------------------------------------------- */

static bool callsign_ok(const char *cs)
{
    /* routes only make sense for flight numbers like BAW123 / UAL1 */
    int n = strlen(cs);
    if (n < 3) return false;
    for (int i = 0; i < n; i++) if (!isalnum((unsigned char)cs[i])) return false;
    return true;
}

/* returns true if an HTTP request was made */
static bool lookup_one(uint32_t addr, const char *callsign)
{
    char url[160];
    bool did_http = false;
    ac_cache_t *ac = ac_cache_find(addr);
    route_cache_t *rt = callsign_ok(callsign) ? route_cache_find(callsign) : NULL;
    bool need_route = callsign_ok(callsign) && !rt;

    if (!ac) {
        if (need_route) snprintf(url, sizeof(url), API_BASE "/aircraft/%06lX?callsign=%s", (unsigned long)addr, callsign);
        else snprintf(url, sizeof(url), API_BASE "/aircraft/%06lX", (unsigned long)addr);

        uint8_t *body; int len;
        s_stats.requests++;
        int status = http_get(url, &body, &len, JSON_MAX);
        did_http = true;
        if (status < 0 || status >= 500 || !body) {
            s_stats.errors++;
            if (status >= 0) snprintf(s_stats.last_error, sizeof(s_stats.last_error), "HTTP %d", status);
            free(body);
            publish(addr, NULL, NULL, LOOKUP_FAILED);
            return true;
        }
        ac = ac_cache_new(addr);
        cJSON *root = cJSON_Parse((char *)body);
        free(body);
        const cJSON *resp = root ? cJSON_GetObjectItemCaseSensitive(root, "response") : NULL;
        if (cJSON_IsObject(resp)) {
            const cJSON *a = cJSON_GetObjectItemCaseSensitive(resp, "aircraft");
            if (cJSON_IsObject(a)) {
                ac->found = true;
                jstr(a, "registration", ac->registration, sizeof(ac->registration));
                jstr(a, "icao_type", ac->type_code, sizeof(ac->type_code));
                jstr(a, "type", ac->type_name, sizeof(ac->type_name));
                jstr(a, "manufacturer", ac->manufacturer, sizeof(ac->manufacturer));
                jstr(a, "registered_owner", ac->owner, sizeof(ac->owner));
                jstr(a, "registered_owner_country_name", ac->owner_country, sizeof(ac->owner_country));
                jstr(a, "url_photo_thumbnail", ac->photo_url, sizeof(ac->photo_url));
            }
            const cJSON *fr = cJSON_GetObjectItemCaseSensitive(resp, "flightroute");
            if (need_route && cJSON_IsObject(fr)) {
                rt = route_cache_new(callsign);
                parse_route(fr, rt);
                need_route = false;
            }
        }
        cJSON_Delete(root);
        if (ac->found) s_stats.ok++; else s_stats.not_found++;
    }

    if (need_route) {
        /* aircraft unknown or cached without route: ask for the callsign alone */
        if (did_http) vTaskDelay(pdMS_TO_TICKS(REQ_INTERVAL_MS));
        snprintf(url, sizeof(url), API_BASE "/callsign/%s", callsign);
        uint8_t *body; int len;
        s_stats.requests++;
        int status = http_get(url, &body, &len, JSON_MAX);
        did_http = true;
        if (status >= 0 && status < 500 && body) {
            rt = route_cache_new(callsign);
            cJSON *root = cJSON_Parse((char *)body);
            const cJSON *resp = root ? cJSON_GetObjectItemCaseSensitive(root, "response") : NULL;
            const cJSON *fr = cJSON_IsObject(resp) ? cJSON_GetObjectItemCaseSensitive(resp, "flightroute") : NULL;
            if (cJSON_IsObject(fr)) parse_route(fr, rt);
            cJSON_Delete(root);
        } else {
            s_stats.errors++;
        }
        free(body);
    }

    lookup_state_t st = (ac && ac->found) || (rt && rt->found) ? LOOKUP_DONE : LOOKUP_NOT_FOUND;
    publish(addr, ac, rt, st);
    return did_http;
}

static void fetch_photo(uint32_t addr)
{
    ac_cache_t *ac = ac_cache_find(addr);
    if (!ac || !ac->photo_url[0]) return;
    uint8_t *body; int len;
    int status = http_get(ac->photo_url, &body, &len, PHOTO_MAX);
    if (status != 200 || !body || len < 100 || body[0] != 0xFF || body[1] != 0xD8) {
        ESP_LOGW(TAG, "photo fetch failed (%d, %d bytes)", status, len);
        free(body);
        return;
    }
    xSemaphoreTake(s_photo_lock, portMAX_DELAY);
    free(s_photo_buf);
    s_photo_buf = body;
    s_photo_len = len;
    s_photo_addr = addr;
    xSemaphoreGive(s_photo_lock);
}

/* pick the next aircraft to enrich: the selected one first, then the closest */
static bool pick_next(uint32_t *addr, char *callsign)
{
    bool found = false;
    double best = 1e12;
    uint32_t t = now_ms();
    uint32_t prio = s_priority;

    aircraft_lock();
    int n;
    aircraft_t *tab = aircraft_table(&n);
    for (int i = 0; i < n; i++) {
        aircraft_t *a = &tab[i];
        if (!a->used || (a->addr & 0xFF000000)) continue;
        bool retry_failed = a->lookup == LOOKUP_FAILED && t - a->lookup_ms > ERROR_BACKOFF_MS;
        if (a->lookup != LOOKUP_NONE && !retry_failed) continue;
        if (a->msgs < 3) continue;
        /* give the callsign a chance to arrive so the route comes back in the same request */
        if (!a->callsign[0] && t - a->first_seen_ms < 15000 && a->addr != prio) continue;
        double score = a->addr == prio ? -1 : (a->dist_valid ? a->dist_nm : 1e6 - (t - a->last_seen_ms));
        if (score < best) {
            best = score;
            *addr = a->addr;
            strlcpy(callsign, a->callsign, 9);
            found = true;
        }
    }
    if (found) {
        aircraft_t *a = aircraft_find(*addr);
        if (a) { a->lookup = LOOKUP_PENDING; a->lookup_ms = t; }
    }
    aircraft_unlock();
    return found;
}

static void lookup_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (!settings_get()->online_lookup || !wifi_is_connected()) continue;

        uint32_t prio = s_priority;
        if (prio && prio != s_photo_addr_done && ac_cache_find(prio)) {
            s_photo_addr_done = prio;
            fetch_photo(prio);
            vTaskDelay(pdMS_TO_TICKS(REQ_INTERVAL_MS));
            continue;
        }

        uint32_t addr;
        char cs[9];
        if (!pick_next(&addr, cs)) continue;
        if (lookup_one(addr, cs)) {
            vTaskDelay(pdMS_TO_TICKS(REQ_INTERVAL_MS));   /* be polite to the free API */
        }
    }
}

void lookup_start(void)
{
    s_ac_cache = heap_caps_calloc(AC_CACHE_N, sizeof(ac_cache_t), MALLOC_CAP_SPIRAM);
    s_route_cache = heap_caps_calloc(ROUTE_CACHE_N, sizeof(route_cache_t), MALLOC_CAP_SPIRAM);
    s_photo_lock = xSemaphoreCreateMutex();
    assert(s_ac_cache && s_route_cache && s_photo_lock);
    xTaskCreatePinnedToCore(lookup_task, "lookup", 8192, NULL, 3, NULL, 1);
}
