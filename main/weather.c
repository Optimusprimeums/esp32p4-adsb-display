#include "weather.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "aircraft.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "settings.h"
#include "wifi.h"

static const char *TAG = "weather";

#define RX_BUF          2048
#define LINE_MAX_LEN    1024
#define IDLE_TIMEOUT_MS 180000     /* stations send every ~20-60 s */
#define LOCK_FALLBACK_MS 90000     /* thermometer-only fallback lock */

static wx_state_t *s_wx;           /* PSRAM */
static SemaphoreHandle_t s_lock;
static volatile bool s_restart;

/* history accumulation */
static uint32_t s_hist_t0;
static float s_interval_gust;
static uint32_t s_first_msg_ms;

/* rain-today baseline, kept in NVS so a reboot doesn't reset "today" */
static int32_t s_rain_day = -1;
static float s_rain_base;
static int32_t s_hilo_day = -1;

const char *wx_link_str(wx_link_t l)
{
    switch (l) {
    case WX_OFF:        return "Off";
    case WX_WAIT_WIFI:  return "Waiting for Wi-Fi";
    case WX_CONNECTING: return "Connecting";
    case WX_CONNECTED:  return "Connected";
    case WX_ERROR:      return "Error";
    }
    return "?";
}

const char *wx_cardinal(float deg)
{
    static const char *n[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                              "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int i = (int)floorf(fmodf(deg + 11.25f + 360.0f, 360.0f) / 22.5f) % 16;
    return n[i];
}

static void set_link(wx_link_t l, const char *detail)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_wx->link = l;
    strlcpy(s_wx->detail, detail ? detail : "", sizeof(s_wx->detail));
    xSemaphoreGive(s_lock);
}

void weather_get(wx_state_t *out)
{
    if (!s_lock) {                 /* client not started (e.g. demo build) */
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(out, s_wx, sizeof(*out));
    xSemaphoreGive(s_lock);
}

void weather_restart(void)
{
    s_restart = true;
}

/* ------------------------------------------------------------------------- */
/* Derived values                                                            */
/* ------------------------------------------------------------------------- */

static float dew_point(float t, float rh)
{
    const float a = 17.62f, b = 243.12f;   /* Magnus, -45..60 C */
    float g = logf(fmaxf(rh, 1.0f) / 100.0f) + a * t / (b + t);
    return b * g / (a - g);
}

static float feels_like(float t, float rh, bool rh_v, float wind_kmh, bool wind_v)
{
    if (rh_v && t >= 27.0f && rh >= 40.0f) {
        /* NWS heat index (Rothfusz), in F */
        float f = t * 9.0f / 5.0f + 32.0f;
        float hi = -42.379f + 2.04901523f * f + 10.14333127f * rh - 0.22475541f * f * rh
                   - 0.00683783f * f * f - 0.05481717f * rh * rh + 0.00122874f * f * f * rh
                   + 0.00085282f * f * rh * rh - 0.00000199f * f * f * rh * rh;
        return (hi - 32.0f) * 5.0f / 9.0f;
    }
    if (wind_v && t <= 10.0f && wind_kmh > 4.8f) {
        float v = powf(wind_kmh, 0.16f);   /* Environment Canada wind chill */
        return 13.12f + 0.6215f * t - 11.37f * v + 0.3965f * t * v;
    }
    return t;
}

static int32_t local_day(void)
{
    if (!time_is_synced()) return -1;
    time_t now = time(NULL);
    struct tm lt;
    localtime_r(&now, &lt);
    return (lt.tm_year + 1900) * 1000 + lt.tm_yday;
}

static void rain_baseline_load(void)
{
    nvs_handle_t h;
    if (nvs_open("weather", NVS_READONLY, &h) != ESP_OK) return;
    uint32_t bits;
    if (nvs_get_i32(h, "rday", &s_rain_day) == ESP_OK && nvs_get_u32(h, "rbase", &bits) == ESP_OK) {
        memcpy(&s_rain_base, &bits, sizeof(bits));
    } else {
        s_rain_day = -1;
    }
    nvs_close(h);
}

static void rain_baseline_save(void)
{
    nvs_handle_t h;
    if (nvs_open("weather", NVS_READWRITE, &h) != ESP_OK) return;
    uint32_t bits;
    memcpy(&bits, &s_rain_base, sizeof(bits));
    nvs_set_i32(h, "rday", s_rain_day);
    nvs_set_u32(h, "rbase", bits);
    nvs_commit(h);
    nvs_close(h);
}

/* called with the lock held after each accepted reading */
static void derive(wx_state_t *w)
{
    w->dew_v = w->temp_v && w->hum_v;
    if (w->dew_v) w->dew_c = dew_point(w->temp_c, w->hum);
    w->feels_v = w->temp_v;
    if (w->feels_v) w->feels_c = feels_like(w->temp_c, w->hum, w->hum_v, w->wind_kmh, w->wind_v);

    int32_t day = local_day();
    w->day_v = day >= 0;
    if (day < 0) return;

    /* daily high / low / max gust since local midnight (or since boot) */
    if (day != s_hilo_day) {
        s_hilo_day = day;
        w->temp_hi = -1000.0f;
        w->temp_lo = 1000.0f;
        w->gust_max_kmh = 0;
    }
    if (w->temp_v) {
        if (w->temp_c > w->temp_hi) w->temp_hi = w->temp_c;
        if (w->temp_c < w->temp_lo) w->temp_lo = w->temp_c;
    }
    if (w->gust_v && w->gust_kmh > w->gust_max_kmh) w->gust_max_kmh = w->gust_kmh;

    /* rain today: the sensor reports a running total */
    if (w->rain_v) {
        if (day != s_rain_day || w->rain_total_mm < s_rain_base) {
            /* new day, or the counter was reset (battery change) */
            s_rain_day = day;
            s_rain_base = w->rain_total_mm;
            rain_baseline_save();
        }
        w->rain_today_mm = w->rain_total_mm - s_rain_base;

        /* rate: change over the last ~15 minutes of history, per hour */
        w->rain_rate_mmh = 0;
        int n = w->hist_n;
        if (n >= 4) {
            const wx_sample_t *old = &w->hist[n - 3];
            if (old->valid && w->rain_total_mm >= old->rain_total_mm) {
                w->rain_rate_mmh = (w->rain_total_mm - old->rain_total_mm) * 4.0f;
            }
        }
    }
}

static void history_tick(wx_state_t *w, uint32_t t)
{
    if (w->gust_v && w->gust_kmh > s_interval_gust) s_interval_gust = w->gust_kmh;
    if (s_hist_t0 == 0) s_hist_t0 = t;
    if (t - s_hist_t0 < WX_HIST_STEP_S * 1000u) return;
    s_hist_t0 = t;

    wx_sample_t smp = {
        .temp_c = w->temp_c, .hum = w->hum, .wind_kmh = w->wind_kmh,
        .gust_kmh = s_interval_gust, .rain_total_mm = w->rain_total_mm, .valid = true,
    };
    s_interval_gust = 0;
    if (w->hist_n == WX_HIST_LEN) {
        memmove(&w->hist[0], &w->hist[1], sizeof(wx_sample_t) * (WX_HIST_LEN - 1));
        w->hist_n--;
    }
    w->hist[w->hist_n++] = smp;
}

/* ------------------------------------------------------------------------- */
/* rtl_433 JSON                                                              */
/* ------------------------------------------------------------------------- */

static bool num(const cJSON *o, const char *k, float *v)
{
    const cJSON *x = cJSON_GetObjectItemCaseSensitive(o, k);
    if (cJSON_IsNumber(x)) { *v = (float)x->valuedouble; return true; }
    if (cJSON_IsBool(x)) { *v = cJSON_IsTrue(x) ? 1.0f : 0.0f; return true; }
    if (cJSON_IsString(x) && x->valuestring && x->valuestring[0]) { *v = strtof(x->valuestring, NULL); return true; }
    return false;
}

static void str_of(const cJSON *o, const char *k, char *dst, size_t n)
{
    const cJSON *x = cJSON_GetObjectItemCaseSensitive(o, k);
    if (cJSON_IsString(x) && x->valuestring) strlcpy(dst, x->valuestring, n);
    else if (cJSON_IsNumber(x)) snprintf(dst, n, "%d", x->valueint);
    else dst[0] = 0;
}

/* first matching field, converted to the given unit */
static bool speed_kmh(const cJSON *o, const char *base, float *out)
{
    char k[32];
    float v;
    snprintf(k, sizeof(k), "%s_km_h", base); if (num(o, k, &v)) { *out = v; return true; }
    snprintf(k, sizeof(k), "%s_kmh", base);  if (num(o, k, &v)) { *out = v; return true; }
    snprintf(k, sizeof(k), "%s_m_s", base);  if (num(o, k, &v)) { *out = v * 3.6f; return true; }
    snprintf(k, sizeof(k), "%s_mi_h", base); if (num(o, k, &v)) { *out = v * 1.609344f; return true; }
    snprintf(k, sizeof(k), "%s_kn", base);   if (num(o, k, &v)) { *out = v * 1.852f; return true; }
    return false;
}

/* Parse one event. Two formats are accepted:
 *  - rtl_433 (HTTP /stream): "model", "id", "temperature_C", "humidity", "wind_avg_km_h", ...
 *  - the RF controller's weather feed (raw TCP): "type":"reading", "model", "station",
 *    "temperature_c", "humidity_percent", "wind_avg_kmh", "wind_gust_kmh", "uv_index",
 *    "rssi_dbm", "age_s"; a "type":"waiting" line comes before its first packet.
 * Returns true if it was accepted for our station. */
static bool handle_event(const char *line, uint32_t t)
{
    cJSON *o = cJSON_Parse(line);
    if (!o) return false;

    char type[16];
    str_of(o, "type", type, sizeof(type));
    if (type[0] && strcmp(type, "reading") != 0) { cJSON_Delete(o); return false; }

    char model[32], id[16], chan[8];
    str_of(o, "model", model, sizeof(model));
    str_of(o, "id", id, sizeof(id));
    if (!id[0]) str_of(o, "station", id, sizeof(id));
    str_of(o, "channel", chan, sizeof(chan));
    if (chan[0]) { strlcat(id, "/", sizeof(id)); strlcat(id, chan, sizeof(id)); }

    float v;
    bool has_temp = num(o, "temperature_C", &v) || num(o, "temperature_c", &v) || num(o, "temperature_F", &v);
    bool has_wx = speed_kmh(o, "wind_avg", &v) || speed_kmh(o, "wind_max", &v) || speed_kmh(o, "wind_speed", &v) ||
                  speed_kmh(o, "wind_gust", &v) ||
                  num(o, "rain_mm", &v) || num(o, "rain_in", &v);
    if (!model[0] || (!has_temp && !has_wx)) { cJSON_Delete(o); return false; }

    bool accepted = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    wx_state_t *w = s_wx;
    if (!w->locked) {
        if (s_first_msg_ms == 0) s_first_msg_ms = t;
        /* prefer a real weather station; settle for a thermometer after a while */
        if (has_wx || t - s_first_msg_ms > LOCK_FALLBACK_MS) {
            w->locked = true;
            strlcpy(w->model, model, sizeof(w->model));
            strlcpy(w->id, id, sizeof(w->id));
            ESP_LOGI(TAG, "locked onto %s id %s", model, id);
        }
    }
    if (w->locked && strcmp(model, w->model) == 0 && strcmp(id, w->id) == 0) {
        accepted = true;
        w->msgs++;
        /* the RF controller repeats its last reading every 30 s with a growing age */
        w->last_ms = (num(o, "age_s", &v) && v > 0 && v * 1000.0f < (float)t) ? t - (uint32_t)(v * 1000.0f) : t;
        if (num(o, "temperature_C", &v) || num(o, "temperature_c", &v)) { w->temp_c = v; w->temp_v = true; }
        else if (num(o, "temperature_F", &v)) { w->temp_c = (v - 32.0f) * 5.0f / 9.0f; w->temp_v = true; }
        if (num(o, "humidity", &v) || num(o, "humidity_percent", &v)) { w->hum = v; w->hum_v = true; }
        if (speed_kmh(o, "wind_avg", &v) || speed_kmh(o, "wind_speed", &v)) { w->wind_kmh = v; w->wind_v = true; }
        if (speed_kmh(o, "wind_max", &v) || speed_kmh(o, "wind_gust", &v)) { w->gust_kmh = v; w->gust_v = true; }
        if (num(o, "wind_dir_deg", &v)) { w->dir_deg = v; w->dir_v = true; }
        if (num(o, "rain_mm", &v)) { w->rain_total_mm = v; w->rain_v = true; }
        else if (num(o, "rain_in", &v)) { w->rain_total_mm = v * 25.4f; w->rain_v = true; }
        if (num(o, "uvi", &v) || num(o, "uv", &v) || num(o, "uv_index", &v)) { w->uvi = v; w->uv_v = true; }
        if (num(o, "light_lux", &v) || num(o, "lux", &v)) { w->lux = v; w->lux_v = true; }
        if (num(o, "pressure_hPa", &v)) { w->press_hpa = v; w->press_v = true; }
        else if (num(o, "pressure_kPa", &v)) { w->press_hpa = v * 10.0f; w->press_v = true; }
        if (num(o, "battery_ok", &v)) { w->batt_ok = v != 0; w->batt_v = true; }
        if (num(o, "rssi", &v) || num(o, "rssi_dbm", &v)) { w->rssi_db = v; w->rssi_v = true; }
        derive(w);
        history_tick(w, t);
    } else {
        w->ignored++;
    }
    xSemaphoreGive(s_lock);
    cJSON_Delete(o);
    return accepted;
}

/* ------------------------------------------------------------------------- */
/* Connection                                                                */
/* ------------------------------------------------------------------------- */

static int connect_to(const char *host, uint16_t port)
{
    char port_s[8], detail[96];
    snprintf(port_s, sizeof(port_s), "%u", port);
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *res = NULL;
    snprintf(detail, sizeof(detail), "%s:%u", host, port);
    set_link(WX_CONNECTING, detail);
    if (getaddrinfo(host, port_s, &hints, &res) != 0 || !res) {
        set_link(WX_ERROR, "DNS lookup failed");
        return -1;
    }
    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) { freeaddrinfo(res); set_link(WX_ERROR, "socket() failed"); return -1; }
    struct timeval tv = {.tv_sec = 5};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int r = connect(sock, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (r < 0) {
        snprintf(detail, sizeof(detail), "connect: %s", strerror(errno));
        set_link(WX_ERROR, detail);
        close(sock);
        return -1;
    }
    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    struct timeval rcv = {.tv_sec = 1};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv, sizeof(rcv));

    char req[160];
    int n = snprintf(req, sizeof(req), "GET /stream HTTP/1.0\r\nHost: %s\r\nAccept: */*\r\n\r\n", host);
    if (send(sock, req, n, 0) != n) {
        set_link(WX_ERROR, "request failed");
        close(sock);
        return -1;
    }
    return sock;
}

static void weather_task(void *arg)
{
    char *buf = malloc(RX_BUF);
    char *line = malloc(LINE_MAX_LEN);
    int backoff = 2000;

    for (;;) {
        s_restart = false;
        const app_settings_t *cfg = settings_get();
        if (!cfg->wx_host[0]) {
            set_link(WX_OFF, "No weather server set");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!wifi_is_connected()) {
            set_link(WX_WAIT_WIFI, "");
            wifi_wait_connected(pdMS_TO_TICKS(1000));
            continue;
        }
        char host[64];
        strlcpy(host, cfg->wx_host, sizeof(host));
        uint16_t port = cfg->wx_port;

        int sock = connect_to(host, port);
        if (sock < 0) {
            for (int t = 0; t < backoff && !s_restart; t += 100) vTaskDelay(pdMS_TO_TICKS(100));
            backoff = backoff < 30000 ? backoff * 2 : 30000;
            continue;
        }
        char detail[96];
        snprintf(detail, sizeof(detail), "%s:%u", host, port);
        set_link(WX_CONNECTED, detail);
        ESP_LOGI(TAG, "connected to weather feed at %s", detail);
        backoff = 2000;

        bool in_headers = true;
        int len = 0;
        uint32_t last_rx = now_ms();
        while (!s_restart) {
            int n = recv(sock, buf, RX_BUF, 0);
            if (n > 0) {
                last_rx = now_ms();
                for (int i = 0; i < n; i++) {
                    char c = buf[i];
                    if (c == '\n') {
                        while (len && line[len - 1] == '\r') len--;
                        line[len] = 0;
                        if (in_headers && line[0] == '{') in_headers = false;   /* raw TCP feed: no HTTP */
                        if (in_headers) {
                            if (len == 0) in_headers = false;          /* end of HTTP headers */
                            else if (strncmp(line, "HTTP/", 5) == 0 && !strstr(line, " 200")) {
                                snprintf(detail, sizeof(detail), "server: %.60s", line);
                                set_link(WX_ERROR, detail);
                                s_restart = true;
                            }
                        } else if (len > 0) {
                            handle_event(line, now_ms());
                        }
                        len = 0;
                    } else if (len < LINE_MAX_LEN - 1) {
                        line[len++] = c;
                    }
                }
            } else if (n == 0) {
                set_link(WX_ERROR, "Server closed connection");
                break;
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (now_ms() - last_rx > IDLE_TIMEOUT_MS) {
                    set_link(WX_ERROR, "No data for 3 minutes");
                    break;
                }
                if (!wifi_is_connected()) break;
            } else {
                snprintf(detail, sizeof(detail), "recv: %s", strerror(errno));
                set_link(WX_ERROR, detail);
                break;
            }
        }
        close(sock);
        if (!s_restart) vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ------------------------------------------------------------------------- */

static void parser_selftest(void)
{
    /* a reading in rtl_433's output format for the Vevor 7-in-1 */
    static const char sample[] =
        "{\"time\":\"2026-11-04 10:00:00\",\"model\":\"Vevor-7in1\",\"id\":63735,\"channel\":0,"
        "\"battery_ok\":1,\"temperature_C\":23.5,\"humidity\":50,\"wind_avg_km_h\":12.4,"
        "\"wind_max_km_h\":18.4,\"rain_mm\":12.582,\"wind_dir_deg\":270,\"uvi\":3,"
        "\"light_lux\":51620,\"mic\":\"CHECKSUM\"}";
    bool ok = handle_event(sample, 1);
    wx_state_t *w = s_wx;
    ok = ok && w->locked && strcmp(w->model, "Vevor-7in1") == 0 && fabsf(w->temp_c - 23.5f) < 0.01f &&
         w->hum_v && fabsf(w->gust_kmh - 18.4f) < 0.01f && w->dir_v && fabsf(w->rain_total_mm - 12.582f) < 0.01f &&
         w->uv_v && w->lux_v && w->batt_ok && w->dew_v && fabsf(w->dew_c - 12.4f) < 0.5f;
    memset(s_wx, 0, sizeof(*s_wx));
    s_first_msg_ms = 0;
    s_hist_t0 = 0;
    s_hilo_day = -1;

    /* the RF controller's raw TCP feed: a "waiting" line, then a reading */
    static const char rf_wait[] = "{\"type\":\"waiting\",\"receiver\":\"running\",\"frequency_mhz\":915.000}";
    static const char rf[] =
        "{\"type\":\"reading\",\"model\":\"VEVOR 7-in-1\",\"station\":\"faf8\",\"channel\":0,\"age_s\":9,"
        "\"stale\":false,\"rssi_dbm\":-90,\"battery_ok\":true,\"temperature_c\":25.7,\"humidity_percent\":66,"
        "\"wind_avg_kmh\":4.7,\"wind_gust_kmh\":7.2,\"wind_dir_deg\":85,\"rain_mm\":231.1,\"uv_index\":0,"
        "\"light_lux\":1689}";
    ok = ok && !handle_event(rf_wait, 100000) && handle_event(rf, 100000);
    ok = ok && w->locked && strcmp(w->id, "faf8/0") == 0 && fabsf(w->temp_c - 25.7f) < 0.01f &&
         fabsf(w->hum - 66) < 0.01f && fabsf(w->wind_kmh - 4.7f) < 0.01f && fabsf(w->gust_kmh - 7.2f) < 0.01f &&
         w->uv_v && w->batt_v && w->batt_ok && w->rssi_v && fabsf(w->rssi_db + 90) < 0.01f && w->last_ms == 91000;

    if (ok) ESP_LOGI(TAG, "parser self-test: passed");
    else ESP_LOGE(TAG, "parser self-test: FAILED");
    /* forget the samples */
    memset(s_wx, 0, sizeof(*s_wx));
    s_first_msg_ms = 0;
    s_hist_t0 = 0;
    s_hilo_day = -1;
}

void weather_start(void)
{
    s_wx = heap_caps_calloc(1, sizeof(wx_state_t), MALLOC_CAP_SPIRAM);
    s_lock = xSemaphoreCreateMutex();
    assert(s_wx && s_lock);
    parser_selftest();
    rain_baseline_load();
    xTaskCreatePinnedToCore(weather_task, "weather", 6144, NULL, 4, NULL, 1);
}
