#include "radar.h"

#include <math.h>
#include <string.h>
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "libs/lodepng/lodepng.h"
#include "src/misc/cache/instance/lv_image_cache.h"
#include "wifi.h"
#include "esp_http_server.h"
#include "ota.h"

static const char *TAG = "radar";

#define INDEX_URL     "https://api.rainviewer.com/public/weather-maps.json"
#define INDEX_EVERY_MS (5 * 60 * 1000)   /* new frames appear every 10 min */
#define TILE          256
#define MAX_ZOOM      7                  /* the public API's limit */
#define COLOR_SCHEME  2                  /* the only scheme the public API still serves */
#define N_TILES       24                 /* decoded tile cache (256 KB each, PSRAM) */
#define TILE_MAX_PNG  (192 * 1024)
#define OVERLAY_OPA   190                /* radar over the map, 0..255 */
#define SETTLE_MS     400                /* re-render once the map stops moving */

typedef struct {
    bool used, missing;     /* missing: fetch failed or no data, draw nothing */
    uint8_t z;
    int32_t x, y;
    uint32_t stamp;         /* LRU */
    uint8_t *rgba;          /* TILE*TILE*4 */
} tile_t;

static SemaphoreHandle_t s_lock;
static volatile bool s_enabled = true;

/* view requested by the UI */
static radar_view_t s_want;
static int64_t s_want_at;
static uint32_t s_want_seq;

/* frame */
static char s_host[64], s_path[64];
static time_t s_frame_time;

/* double-buffered output: the task renders into s_buf[s_back]; radar_take()
 * (LVGL task) swaps it to the front once s_ready is set */
static uint8_t *s_buf[2];
static lv_image_dsc_t s_dsc[2];
static int s_back = 1, s_front = -1;
static volatile bool s_ready;
static radar_view_t s_ready_view, s_front_view;
static int32_t s_bw, s_bh;

static tile_t s_tiles[N_TILES];
static uint32_t s_stamp;
static char s_status[64] = "Loading radar...";

/* diagnostics for GET /radar */
static struct {
    int index_status, index_count;
    int tile_ok, tile_empty, tile_http_err, tile_png_err, last_tile_status;
    unsigned last_png_err;
    esp_err_t last_open_err;
    char last_url[160];
    int last_zoom, last_need, last_got;
    uint32_t renders, last_render_ms, last_draw_ms;
    uint32_t takes;
} s_diag;

#define LOCK()   xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

static void set_status(const char *s)
{
    LOCK();
    strlcpy(s_status, s, sizeof(s_status));
    UNLOCK();
}

/* GET url into a PSRAM buffer; returns the HTTP status or -1 */
static int http_get(const char *url, uint8_t **out, int *len, int max)
{
    *out = NULL;
    *len = 0;
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 8000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 2048,
        .user_agent = "esp32p4-adsb-display/1.0",
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return -1;
    int st = -1;
    esp_err_t oe = esp_http_client_open(c, 0);
    s_diag.last_open_err = oe;
    if (oe == ESP_OK) {
        esp_http_client_fetch_headers(c);
        st = esp_http_client_get_status_code(c);
        uint8_t *b = heap_caps_malloc(max + 1, MALLOC_CAP_SPIRAM);
        int n = 0;
        if (b) {
            while (n < max) {
                int r = esp_http_client_read(c, (char *)b + n, max - n);
                if (r <= 0) break;
                n += r;
            }
            b[n] = 0;
            *out = b;
            *len = n;
        }
    }
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    return st;
}

/* newest past radar frame; true if it changed */
static bool fetch_index(void)
{
    uint8_t *b;
    int n;
    int st = http_get(INDEX_URL, &b, &n, 16 * 1024);
    s_diag.index_status = st;
    s_diag.index_count++;
    if (st != 200 || !b) {
        char m[64];
        snprintf(m, sizeof(m), st < 0 ? "Radar: no internet" : "Radar: RainViewer said %d", st);
        set_status(m);
        free(b);
        return false;
    }
    bool changed = false;
    cJSON *j = cJSON_Parse((char *)b);
    free(b);
    const cJSON *host = cJSON_GetObjectItem(j, "host");
    const cJSON *radar = cJSON_GetObjectItem(j, "radar");
    const cJSON *past = radar ? cJSON_GetObjectItem(radar, "past") : NULL;
    int np = cJSON_GetArraySize(past);
    const cJSON *last = np ? cJSON_GetArrayItem(past, np - 1) : NULL;
    const cJSON *path = last ? cJSON_GetObjectItem(last, "path") : NULL;
    const cJSON *t = last ? cJSON_GetObjectItem(last, "time") : NULL;
    if (cJSON_IsString(host) && cJSON_IsString(path) && cJSON_IsNumber(t)) {
        if (strcmp(s_path, path->valuestring) != 0) {
            strlcpy(s_host, host->valuestring, sizeof(s_host));
            strlcpy(s_path, path->valuestring, sizeof(s_path));
            s_frame_time = (time_t)t->valuedouble;
            changed = true;
            for (int i = 0; i < N_TILES; i++) s_tiles[i].used = false;   /* new frame: old tiles are stale */
            ESP_LOGI(TAG, "frame %s", s_path);
        }
    } else {
        set_status("Radar: unexpected reply from RainViewer");
    }
    cJSON_Delete(j);
    return changed;
}

static tile_t *tile_get(int z, int32_t x, int32_t y)
{
    int n = 1 << z;
    if (y < 0 || y >= n) return NULL;
    x = ((x % n) + n) % n;
    tile_t *free_t = NULL, *old = NULL;
    for (int i = 0; i < N_TILES; i++) {
        tile_t *t = &s_tiles[i];
        if (t->used && t->z == z && t->x == x && t->y == y) {
            t->stamp = ++s_stamp;
            return t;
        }
        if (!t->used && !free_t) free_t = t;
        if (t->used && (!old || t->stamp < old->stamp)) old = t;
    }
    tile_t *t = free_t ? free_t : old;
    t->used = true;
    t->missing = true;
    t->z = z;
    t->x = x;
    t->y = y;
    t->stamp = ++s_stamp;

    char url[192];
    snprintf(url, sizeof(url), "%s%s/%d/%d/%ld/%ld/%d/1_0.png", s_host, s_path, TILE, z, (long)x, (long)y, COLOR_SCHEME);
    uint8_t *png;
    int len;
    int st = http_get(url, &png, &len, TILE_MAX_PNG);
    s_diag.last_tile_status = st;
    strlcpy(s_diag.last_url, url, sizeof(s_diag.last_url));
    if (st == 200 && png) {
        /* LVGL's copy of lodepng hands back an lv_draw_buf_t (pixels still in R G B A order),
           not a plain pixel array */
        unsigned char *out = NULL;
        unsigned w, h;
        unsigned err = lodepng_decode32(&out, &w, &h, png, len);
        lv_draw_buf_t *db = (lv_draw_buf_t *)out;
        if (!err && db && w == TILE && h == TILE && db->data && db->header.stride >= TILE * 4) {
            for (int r = 0; r < TILE; r++) {
                memcpy(t->rgba + (size_t)r * TILE * 4, db->data + (size_t)r * db->header.stride, TILE * 4);
            }
            t->missing = false;
            s_diag.tile_ok++;
            if (len < 400) s_diag.tile_empty++;
        } else {
            ESP_LOGW(TAG, "tile %d/%ld/%ld: bad png (%u)", z, (long)x, (long)y, err);
            s_diag.tile_png_err++;
            s_diag.last_png_err = err;
        }
        if (db) lv_draw_buf_destroy(db);
    } else {
        ESP_LOGW(TAG, "tile %d/%ld/%ld: HTTP %d", z, (long)x, (long)y, st);
        s_diag.tile_http_err++;
    }
    free(png);
    return t;
}

static double merc_y(double lat)
{
    double r = lat * M_PI / 180.0;
    return (1.0 - log(tan(r) + 1.0 / cos(r)) / M_PI) / 2.0;   /* 0 (north) .. 1 (south) */
}

/* pick the zoom whose pixels are closest to the map's, within the API's range */
static int pick_zoom(const radar_view_t *v)
{
    double nm_per_px = 1.0 / v->px_per_nm;
    double coslat = cos(v->clat * M_PI / 180.0);
    /* nm per tile pixel at zoom z: 360*60*coslat / (256 * 2^z) */
    double z = log2(360.0 * 60.0 * coslat / (TILE * nm_per_px));
    int zi = (int)floor(z + 0.3);
    if (zi < 1) zi = 1;
    if (zi > MAX_ZOOM) zi = MAX_ZOOM;
    return zi;
}

/* Reproject the radar tiles into s_buf[s_back] for view v (same flat projection
 * as the map: x scales with cos(centre latitude), y with latitude). */
static void render(const radar_view_t *v)
{
    int z = pick_zoom(v);
    double world = (double)TILE * (1 << z);
    double coslat = cos(v->clat * M_PI / 180.0);
    double cx = v->w / 2.0, cy = v->h / 2.0;
    uint8_t *out = s_buf[s_back];

    /* fetch every tile the view touches first (corners + edges cover the range) */
    double lat_top = v->clat + cy / v->px_per_nm / 60.0, lat_bot = v->clat - cy / v->px_per_nm / 60.0;
    if (lat_top > 85) lat_top = 85;
    if (lat_bot < -85) lat_bot = -85;
    double lon_l = v->clon - cx / v->px_per_nm / 60.0 / coslat, lon_r = v->clon + cx / v->px_per_nm / 60.0 / coslat;
    int32_t tx0 = (int32_t)floor((lon_l + 180.0) / 360.0 * (1 << z)), tx1 = (int32_t)floor((lon_r + 180.0) / 360.0 * (1 << z));
    int32_t ty0 = (int32_t)floor(merc_y(lat_top) * (1 << z)), ty1 = (int32_t)floor(merc_y(lat_bot) * (1 << z));
    int need = (tx1 - tx0 + 1) * (ty1 - ty0 + 1);
    if (need > N_TILES) {
        set_status("Radar: zoom in to show radar");
        memset(out, 0, (size_t)v->w * v->h * 4);
        return;
    }
    int got = 0;
    for (int32_t ty = ty0; ty <= ty1; ty++) {
        for (int32_t tx = tx0; tx <= tx1; tx++) {
            tile_t *t = tile_get(z, tx, ty);
            if (t && !t->missing) got++;
        }
    }

    /* The P4's FPU is single precision, so doubles are software-emulated: keep them
     * per row only. Across a row the world x is linear, so it is stepped in 16.16
     * fixed point relative to the left tile column (tx0). */
    int64_t td = esp_timer_get_time();
    double dlon = 1.0 / v->px_per_nm / 60.0 / coslat;
    double wx0 = (lon_l + 180.0) / 360.0 * world - (double)tx0 * TILE, dwx = dlon / 360.0 * world;
    uint32_t fx0 = (uint32_t)((wx0 + dwx * 0.5) * 65536.0), dfx = (uint32_t)(dwx * 65536.0);
    int ncols = tx1 - tx0 + 1, n = 1 << z;
    int32_t prev_ty = INT32_MIN, prev_iy = -1;
    const uint32_t *prev_row = NULL;
    for (int32_t py = 0; py < v->h; py++) {
        double lat = v->clat + (cy - py - 0.5) / v->px_per_nm / 60.0;
        uint32_t *row = (uint32_t *)(out + (size_t)py * v->w * 4);
        if (lat > 85 || lat < -85) {
            memset(row, 0, v->w * 4);
            prev_row = NULL;
            continue;
        }
        double wy = merc_y(lat) * world;
        int32_t ty = (int32_t)floor(wy / TILE), iy = (int32_t)wy - ty * TILE;
        if (prev_row && ty == prev_ty && iy == prev_iy) {    /* zoomed in: same source row again */
            memcpy(row, prev_row, v->w * 4);
            continue;
        }
        /* this row's tiles, one per column */
        const uint8_t *src[N_TILES];
        bool any = false;
        for (int k = 0; k < ncols; k++) {
            int32_t wtx = (((tx0 + k) % n) + n) % n;
            src[k] = NULL;
            for (int i = 0; i < N_TILES; i++) {
                const tile_t *c = &s_tiles[i];
                if (c->used && !c->missing && c->z == z && c->x == wtx && c->y == ty) {
                    src[k] = c->rgba + (size_t)iy * TILE * 4;
                    any = true;
                    break;
                }
            }
        }
        prev_ty = ty;
        prev_iy = iy;
        prev_row = row;
        if (!any) {
            memset(row, 0, v->w * 4);
            continue;
        }
        uint32_t fx = fx0;
        for (int32_t px = 0; px < v->w; px++, fx += dfx) {
            uint32_t lx = fx >> 16;
            uint32_t k = lx >> 8;
            const uint8_t *s = k < (uint32_t)ncols ? src[k] : NULL;
            if (!s) {
                row[px] = 0;
                continue;
            }
            const uint8_t *p = s + (lx & (TILE - 1)) * 4;   /* R G B A */
            uint32_t a = (uint32_t)p[3] * OVERLAY_OPA >> 8;
            row[px] = a ? (a << 24) | ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2] : 0;   /* LVGL ARGB8888 */
        }
    }
    s_diag.last_draw_ms = (uint32_t)((esp_timer_get_time() - td) / 1000);
    s_diag.last_zoom = z;
    s_diag.last_need = need;
    s_diag.last_got = got;
    if (!got) set_status("Radar: no radar data here");
    else set_status("");
}

static void radar_task(void *arg)
{
    int64_t last_index = 0;
    uint32_t done_seq = 0;
    bool fresh_frame = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_enabled || !wifi_is_connected()) continue;
        int64_t now = esp_timer_get_time() / 1000;
        if (!last_index || now - last_index > INDEX_EVERY_MS) {
            last_index = now;
            if (fetch_index()) fresh_frame = true;
        }
        if (!s_path[0] || s_ready) continue;     /* nothing to draw, or the last image not picked up yet */

        LOCK();
        radar_view_t v = s_want;
        uint32_t seq = s_want_seq;
        int64_t at = s_want_at;
        UNLOCK();
        if (v.w <= 0 || v.h <= 0 || v.w > s_bw || v.h > s_bh) continue;
        if (seq == done_seq && !fresh_frame) continue;
        if (now - at < SETTLE_MS) continue;      /* still panning or zooming */

        int64_t t0 = esp_timer_get_time();
        render(&v);
        s_diag.last_render_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        s_diag.renders++;
        ESP_LOGI(TAG, "rendered %ldx%ld in %lu ms", (long)v.w, (long)v.h, (unsigned long)s_diag.last_render_ms);
        LOCK();
        s_dsc[s_back].header.w = v.w;
        s_dsc[s_back].header.h = v.h;
        s_dsc[s_back].header.stride = v.w * 4;
        s_dsc[s_back].data_size = (uint32_t)v.w * v.h * 4;
        s_ready_view = v;
        s_ready = true;
        UNLOCK();
        done_seq = seq;
        fresh_frame = false;
    }
}

static esp_err_t diag_get(httpd_req_t *req)
{
    char b[900];
    LOCK();
    radar_view_t w = s_want;
    char st[64];
    strlcpy(st, s_status, sizeof(st));
    UNLOCK();
    snprintf(b, sizeof(b),
             "{\"enabled\":%s,\"status\":\"%s\",\"frame\":\"%s\",\"frame_time\":%lld,"
             "\"index_status\":%d,\"index_count\":%d,\"open_err\":\"%s\","
             "\"tile_ok\":%d,\"tile_empty\":%d,\"tile_http_err\":%d,\"tile_png_err\":%d,\"last_png_err\":%u,"
             "\"last_tile_status\":%d,\"last_url_tail\":\"%s\",\"zoom\":%d,\"need\":%d,\"got\":%d,"
             "\"renders\":%lu,\"render_ms\":%lu,\"draw_ms\":%lu,\"takes\":%lu,\"ready\":%s,\"front\":%d,"
             "\"view\":{\"w\":%ld,\"h\":%ld,\"px_per_nm\":%.3f}}",
             s_enabled ? "true" : "false", st, s_path, (long long)s_frame_time, s_diag.index_status, s_diag.index_count,
             esp_err_to_name(s_diag.last_open_err), s_diag.tile_ok, s_diag.tile_empty, s_diag.tile_http_err,
             s_diag.tile_png_err, s_diag.last_png_err, s_diag.last_tile_status,
             strlen(s_diag.last_url) > 40 ? s_diag.last_url + strlen(s_diag.last_url) - 40 : s_diag.last_url,
             s_diag.last_zoom, s_diag.last_need, s_diag.last_got, (unsigned long)s_diag.renders,
             (unsigned long)s_diag.last_render_ms, (unsigned long)s_diag.last_draw_ms, (unsigned long)s_diag.takes, s_ready ? "true" : "false", s_front,
             (long)w.w, (long)w.h, w.px_per_nm);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, b, HTTPD_RESP_USE_STRLEN);
}

void radar_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_bw = 800;
    s_bh = 800;
    for (int i = 0; i < 2; i++) {
        s_buf[i] = heap_caps_calloc((size_t)s_bw * s_bh, 4, MALLOC_CAP_SPIRAM);
        assert(s_buf[i]);
        s_dsc[i].header.magic = LV_IMAGE_HEADER_MAGIC;
        s_dsc[i].header.cf = LV_COLOR_FORMAT_ARGB8888;
        s_dsc[i].data = s_buf[i];
    }
    for (int i = 0; i < N_TILES; i++) {
        s_tiles[i].rgba = heap_caps_malloc(TILE * TILE * 4, MALLOC_CAP_SPIRAM);
        assert(s_tiles[i].rgba);
    }
    xTaskCreatePinnedToCore(radar_task, "radar", 16384, NULL, 3, NULL, 1);
    static const httpd_uri_t uri = {.uri = "/radar", .method = HTTP_GET, .handler = diag_get};
    ota_register_uri(&uri);
}

void radar_enable(bool on)
{
    s_enabled = on;
}

void radar_set_view(const radar_view_t *v)
{
    if (!s_lock) return;
    LOCK();
    if (memcmp(&s_want, v, sizeof(*v)) != 0) {
        s_want = *v;
        s_want_seq++;
        s_want_at = esp_timer_get_time() / 1000;
    }
    UNLOCK();
}

const lv_image_dsc_t *radar_take(radar_view_t *view)
{
    if (!s_lock) return NULL;
    LOCK();
    if (s_ready) {
        int f = s_back;
        s_back = s_front < 0 ? (f ^ 1) : s_front;
        s_front = f;
        s_front_view = s_ready_view;
        s_ready = false;
        s_diag.takes++;
        lv_image_cache_drop(&s_dsc[f]);
    }
    int f = s_front;
    if (view) *view = s_front_view;
    UNLOCK();
    return f < 0 ? NULL : &s_dsc[f];
}

void radar_get_info(radar_info_t *out)
{
    LOCK();
    out->ok = s_front >= 0;
    out->frame_time = s_frame_time;
    strlcpy(out->status, s_status, sizeof(out->status));
    UNLOCK();
}
