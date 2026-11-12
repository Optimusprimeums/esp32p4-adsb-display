#include "airband.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "aircraft.h"
#include "board.h"
#include "cJSON.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "settings.h"
#include "wifi.h"

static const char *TAG = "airband";

/* ES8311 codec + NS4150 amplifier, pins from the Guition JC8012P4A1 BSP */
#define PIN_I2S_MCLK  13
#define PIN_I2S_BCLK  12
#define PIN_I2S_WS    10
#define PIN_I2S_DOUT  9
#define PIN_PA_EN     20

#define RX_BUF        2048
#define EV_LINE_MAX   4096
#define SB_BYTES      (192 * 1024)   /* ~2 s at 48 kHz, ~12 s at 8 kHz */
#define PREBUF_MS     250            /* jitter buffer before a transmission starts playing */
#define MAX_LAG_MS    1500           /* skip ahead when playback falls this far behind */
#define LAG_KEEP_MS   400

static ab_state_t *s_ab;             /* PSRAM */
static SemaphoreHandle_t s_lock;
static volatile bool s_audio_restart, s_event_restart, s_vol_dirty = true;
static volatile uint32_t s_stream_rate;
static StreamBufferHandle_t s_sb;
static esp_codec_dev_handle_t s_codec;

#define LOCK()   xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

const char *ab_link_str(ab_link_t l)
{
    switch (l) {
    case AB_OFF:        return "Off";
    case AB_WAIT_WIFI:  return "Waiting for Wi-Fi";
    case AB_CONNECTING: return "Connecting";
    case AB_CONNECTED:  return "Connected";
    case AB_ERROR:      return "Error";
    }
    return "?";
}

static void set_link(bool audio, ab_link_t l, const char *detail)
{
    LOCK();
    if (audio) {
        s_ab->audio_link = l;
        strlcpy(s_ab->audio_detail, detail ? detail : "", sizeof(s_ab->audio_detail));
    } else {
        s_ab->event_link = l;
        strlcpy(s_ab->event_detail, detail ? detail : "", sizeof(s_ab->event_detail));
    }
    UNLOCK();
}

void airband_get(ab_state_t *out)
{
    if (!s_ab) {
        memset(out, 0, sizeof(*out));
        return;
    }
    LOCK();
    memcpy(out, s_ab, sizeof(*out));
    UNLOCK();
}

void airband_restart(void)
{
    s_audio_restart = true;
    s_event_restart = true;
}

void airband_apply_volume(void)
{
    s_vol_dirty = true;
}

/* ------------------------------------------------------------------------- */
/* Codec                                                                     */
/* ------------------------------------------------------------------------- */

static bool codec_init(void)
{
    i2s_chan_handle_t tx = NULL;
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.dma_desc_num = 6;
    cc.dma_frame_num = 240;
    cc.auto_clear_after_cb = true;      /* silence, not a looping buffer, on underrun */
    if (i2s_new_channel(&cc, &tx, NULL) != ESP_OK) return false;

    i2s_std_config_t sc = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = PIN_I2S_MCLK,
            .bclk = PIN_I2S_BCLK,
            .ws = PIN_I2S_WS,
            .dout = PIN_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    if (i2s_channel_init_std_mode(tx, &sc) != ESP_OK) return false;

    audio_codec_i2s_cfg_t ic = {.port = I2S_NUM_0, .tx_handle = tx};
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&ic);
    audio_codec_i2c_cfg_t i2c = {.port = 0, .addr = ES8311_CODEC_DEFAULT_ADDR, .bus_handle = board_i2c_bus()};
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c);
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (!data_if || !ctrl_if || !gpio_if) return false;

    es8311_codec_cfg_t ec = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = PIN_PA_EN,
        .use_mclk = true,
        .hw_gain = {.pa_voltage = 5.0, .codec_dac_voltage = 3.3},
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&ec);
    if (!codec_if) return false;
    esp_codec_dev_cfg_t dc = {.dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = codec_if, .data_if = data_if};
    s_codec = esp_codec_dev_new(&dc);
    return s_codec != NULL;
}

/* ------------------------------------------------------------------------- */
/* HTTP                                                                      */
/* ------------------------------------------------------------------------- */

static int http_get(bool audio, const char *host, uint16_t port, const char *path)
{
    char port_s[8], detail[96];
    snprintf(port_s, sizeof(port_s), "%u", port);
    snprintf(detail, sizeof(detail), "%s:%u", host, port);
    set_link(audio, AB_CONNECTING, detail);
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, port_s, &hints, &res) != 0 || !res) {
        set_link(audio, AB_ERROR, "DNS lookup failed");
        return -1;
    }
    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        freeaddrinfo(res);
        set_link(audio, AB_ERROR, "socket() failed");
        return -1;
    }
    struct timeval tv = {.tv_sec = 5};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int r = connect(sock, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (r < 0) {
        snprintf(detail, sizeof(detail), "connect: %s", strerror(errno));
        set_link(audio, AB_ERROR, detail);
        close(sock);
        return -1;
    }
    /* Both streams can be silent for hours: let TCP keepalive find dead links */
    int one = 1, idle = 30, intvl = 10, cnt = 3;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    if (audio) setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    struct timeval rcv = {.tv_sec = 1};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv, sizeof(rcv));

    char req[320];
    int n = snprintf(req, sizeof(req),
                     "GET %s HTTP/1.0\r\nHost: %s\r\nAccept: */*\r\nUser-Agent: adsb-display\r\n\r\n", path, host);
    if (n >= (int)sizeof(req) || send(sock, req, n, 0) != n) {
        set_link(audio, AB_ERROR, "request failed");
        close(sock);
        return -1;
    }
    return sock;
}

/* Percent-encode a query value */
static void url_encode(const char *in, char *out, size_t n)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t o = 0;
    for (; *in && o + 4 < n; in++) {
        unsigned char c = (unsigned char)*in;
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || strchr("-_.~", c)) {
            out[o++] = (char)c;
        } else {
            out[o++] = '%';
            out[o++] = hex[c >> 4];
            out[o++] = hex[c & 15];
        }
    }
    out[o] = 0;
}

static void wait_restart(volatile bool *flag, int ms)
{
    for (int t = 0; t < ms && !*flag; t += 100) vTaskDelay(pdMS_TO_TICKS(100));
}

/* ------------------------------------------------------------------------- */
/* Audio stream: WAV over HTTP -> stream buffer                              */
/* ------------------------------------------------------------------------- */

static void audio_task(void *arg)
{
    uint8_t *buf = malloc(RX_BUF);
    char hdr[1024];
    uint8_t wav[44];
    int backoff = 2000;

    for (;;) {
        s_audio_restart = false;
        const app_settings_t *cfg = settings_get();
        if (!cfg->ab_host[0]) {
            set_link(true, AB_OFF, "No GopherTrunk server set");
            wait_restart(&s_audio_restart, 1000);
            continue;
        }
        if (!cfg->ab_listen) {
            set_link(true, AB_OFF, "Not listening");
            wait_restart(&s_audio_restart, 500);
            continue;
        }
        if (!wifi_is_connected()) {
            set_link(true, AB_WAIT_WIFI, "");
            wifi_wait_connected(pdMS_TO_TICKS(1000));
            continue;
        }
        char host[64], path[200], enc[150];
        strlcpy(host, cfg->ab_host, sizeof(host));
        uint16_t port = cfg->ab_port;
        if (cfg->ab_filter[0]) {
            url_encode(cfg->ab_filter, enc, sizeof(enc));
            snprintf(path, sizeof(path), "/api/v1/audio/stream?device=%s", enc);
        } else {
            strlcpy(path, "/api/v1/audio/stream", sizeof(path));
        }

        int sock = http_get(true, host, port, path);
        if (sock < 0) {
            wait_restart(&s_audio_restart, backoff);
            backoff = backoff < 30000 ? backoff * 2 : 30000;
            continue;
        }
        ESP_LOGI(TAG, "audio stream %s:%u%s", host, port, path);

        int hlen = 0, wlen = 0;
        bool in_hdr = true, odd = false, ok = true;
        uint8_t carry = 0;
        char detail[96];
        while (!s_audio_restart && settings_get()->ab_listen) {
            int n = recv(sock, buf, RX_BUF, 0);
            if (n == 0) {
                set_link(true, AB_ERROR, "Server closed the stream");
                ok = false;
                break;
            }
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    if (!wifi_is_connected()) break;
                    continue;
                }
                snprintf(detail, sizeof(detail), "recv: %s", strerror(errno));
                set_link(true, AB_ERROR, detail);
                ok = false;
                break;
            }
            int i = 0;
            /* HTTP response headers */
            while (in_hdr && i < n) {
                if (hlen < (int)sizeof(hdr) - 1) hdr[hlen++] = (char)buf[i];
                i++;
                if (hlen >= 4 && memcmp(hdr + hlen - 4, "\r\n\r\n", 4) == 0) {
                    hdr[hlen] = 0;
                    in_hdr = false;
                    char *eol = strstr(hdr, "\r\n");
                    if (eol) *eol = 0;
                    if (!strstr(hdr, " 200") && !strstr(hdr, " 206")) {
                        if (strstr(hdr, " 503")) {
                            snprintf(detail, sizeof(detail), "Live audio off in GopherTrunk (503)");
                        } else {
                            snprintf(detail, sizeof(detail), "server: %.50s", hdr);
                        }
                        set_link(true, AB_ERROR, detail);
                        ok = false;
                    }
                }
            }
            if (!ok) break;
            /* 44-byte WAV header */
            while (!in_hdr && wlen < 44 && i < n) wav[wlen++] = buf[i++];
            if (wlen == 44 && !in_hdr && s_stream_rate == 0) {
                uint32_t rate = wav[24] | wav[25] << 8 | wav[26] << 16 | (uint32_t)wav[27] << 24;
                uint16_t chans = wav[22] | wav[23] << 8, bits = wav[34] | wav[35] << 8;
                if (memcmp(wav, "RIFF", 4) || memcmp(wav + 8, "WAVE", 4) || chans != 1 || bits != 16 ||
                    rate < 4000 || rate > 48000) {
                    set_link(true, AB_ERROR, "Unexpected audio format");
                    ok = false;
                    break;
                }
                s_stream_rate = rate;
                snprintf(detail, sizeof(detail), "%s:%u, %lu Hz", host, port, (unsigned long)rate);
                set_link(true, AB_CONNECTED, detail);
                LOCK();
                s_ab->rate = rate;
                UNLOCK();
                backoff = 2000;
            }
            if (s_stream_rate == 0 || i >= n) continue;
            /* PCM: keep 16-bit alignment across recv() boundaries */
            uint8_t *p = buf + i;
            int len = n - i;
            if (odd) {
                uint8_t pair[2] = {carry, p[0]};
                xStreamBufferSend(s_sb, pair, 2, 0);
                p++;
                len--;
                odd = false;
            }
            if (len & 1) {
                carry = p[len - 1];
                odd = true;
                len--;
            }
            if (len > 0) {
                size_t sent = xStreamBufferSend(s_sb, p, len, 0);
                (void)sent;   /* overflow: drop; the player skips ahead when it falls behind */
            }
            LOCK();
            s_ab->bytes_rx += n - i;
            UNLOCK();
        }
        close(sock);
        s_stream_rate = 0;
        if (!ok && !s_audio_restart) {
            wait_restart(&s_audio_restart, backoff);
            backoff = backoff < 30000 ? backoff * 2 : 30000;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Player: stream buffer -> ES8311                                           */
/* ------------------------------------------------------------------------- */

static void player_task(void *arg)
{
    const size_t in_max = 960;                      /* bytes = 480 mono samples */
    int16_t *in = heap_caps_malloc(in_max, MALLOC_CAP_INTERNAL);
    int16_t *out = heap_caps_malloc(in_max * 2, MALLOC_CAP_INTERNAL);
    uint32_t open_rate = 0, first_ms = 0;
    bool playing = false;
    float level = 0;

    /* board_init() runs in the LVGL task: wait for the shared I2C bus */
    while (!board_i2c_bus()) vTaskDelay(pdMS_TO_TICKS(100));
    bool ok = codec_init();
    LOCK();
    s_ab->codec_ok = ok;
    UNLOCK();
    if (!ok) ESP_LOGE(TAG, "ES8311 codec init failed: no audio output");
    else ESP_LOGI(TAG, "ES8311 codec ready");

    for (;;) {
        const app_settings_t *cfg = settings_get();
        uint32_t rate = s_stream_rate;
        bool want = cfg->ab_listen && s_codec && rate;
        if (open_rate && (!want || rate != open_rate)) {
            esp_codec_dev_close(s_codec);          /* also switches the amplifier off */
            open_rate = 0;
            playing = false;
        }
        if (!want) {
            if (!cfg->ab_listen) xStreamBufferReset(s_sb);
            LOCK();
            s_ab->playing = false;
            s_ab->level = 0;
            s_ab->buffered_ms = 0;
            UNLOCK();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!open_rate) {
            esp_codec_dev_sample_info_t fs = {.bits_per_sample = 16, .channel = 2, .sample_rate = rate};
            if (esp_codec_dev_open(s_codec, &fs) != ESP_CODEC_DEV_OK) {
                ESP_LOGE(TAG, "codec open at %lu Hz failed", (unsigned long)rate);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            open_rate = rate;
            s_vol_dirty = true;
            ESP_LOGI(TAG, "codec open, %lu Hz", (unsigned long)rate);
        }
        if (s_vol_dirty) {
            s_vol_dirty = false;
            esp_codec_dev_set_out_vol(s_codec, cfg->ab_volume);
            esp_codec_dev_set_out_mute(s_codec, cfg->ab_mute);
        }

        uint32_t bpms = rate * 2 / 1000;               /* bytes per ms */
        size_t avail = xStreamBufferBytesAvailable(s_sb);
        uint32_t t = now_ms();
        if (!playing) {
            if (avail == 0) {
                first_ms = 0;
                level *= 0.7f;
                LOCK();
                s_ab->playing = false;
                s_ab->level = (uint8_t)level;
                s_ab->buffered_ms = 0;
                UNLOCK();
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            if (!first_ms) first_ms = t;
            if (avail < PREBUF_MS * bpms && t - first_ms < PREBUF_MS) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            playing = true;
        }
        if (avail > MAX_LAG_MS * bpms) {
            size_t drop = (avail - LAG_KEEP_MS * bpms) & ~(size_t)1;
            while (drop > 0) {
                size_t k = xStreamBufferReceive(s_sb, in, drop < in_max ? drop : in_max, 0);
                if (!k) break;
                drop -= k;
            }
        }
        size_t n = xStreamBufferReceive(s_sb, in, in_max, pdMS_TO_TICKS(150)) & ~(size_t)1;
        if (n == 0) {
            playing = false;                            /* end of the transmission */
            first_ms = 0;
            continue;
        }
        size_t ns = n / 2;
        int peak = 0;
        for (size_t i = 0; i < ns; i++) {
            int v = in[i];
            int a = v < 0 ? -v : v;
            if (a > peak) peak = a;
            out[2 * i] = out[2 * i + 1] = in[i];
        }
        float lv = sqrtf(peak / 32768.0f) * 100.0f;
        level = lv > level ? lv : level * 0.85f + lv * 0.15f;
        esp_codec_dev_write(s_codec, out, (int)(ns * 4));
        LOCK();
        s_ab->playing = true;
        s_ab->level = (uint8_t)(level > 100 ? 100 : level);
        s_ab->buffered_ms = xStreamBufferBytesAvailable(s_sb) / (bpms ? bpms : 1);
        UNLOCK();
    }
}

/* ------------------------------------------------------------------------- */
/* Call events (SSE)                                                         */
/* ------------------------------------------------------------------------- */

static void fmt_freq_label(uint32_t hz, char *out, size_t n)
{
    snprintf(out, n, "%lu.%03lu", (unsigned long)(hz / 1000000), (unsigned long)(hz / 1000 % 1000));
}

static ab_channel_t *channel_upsert(const char *serial, const char *label, uint32_t freq)
{
    ab_channel_t *c = NULL;
    for (int i = 0; i < s_ab->ch_n; i++) {
        if (strcmp(s_ab->ch[i].serial, serial) == 0) { c = &s_ab->ch[i]; break; }
    }
    if (!c) {
        if (s_ab->ch_n < AB_MAX_CHANNELS) {
            c = &s_ab->ch[s_ab->ch_n++];
        } else {
            /* replace the longest-idle channel that isn't on air */
            for (int i = 0; i < AB_MAX_CHANNELS; i++) {
                ab_channel_t *x = &s_ab->ch[i];
                if (!x->on_air && (!c || x->last < c->last)) c = x;
            }
            if (!c) return NULL;
        }
        memset(c, 0, sizeof(*c));
        strlcpy(c->serial, serial, sizeof(c->serial));
    }
    if (label[0]) strlcpy(c->label, label, sizeof(c->label));
    if (freq) c->freq_hz = freq;
    return c;
}

static void refresh_on_air(void)
{
    /* the most recently keyed channel that is still on air */
    ab_channel_t *best = NULL;
    for (int i = 0; i < s_ab->ch_n; i++) {
        ab_channel_t *c = &s_ab->ch[i];
        if (c->on_air && (!best || c->last >= best->last)) best = c;
    }
    s_ab->on_air = best != NULL;
    if (best) {
        strlcpy(s_ab->label, best->label, sizeof(s_ab->label));
        s_ab->freq_hz = best->freq_hz;
    }
}

static bool handle_call(bool start, const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) return false;
    const cJSON *pl = cJSON_GetObjectItem(root, "payload");
    const cJSON *g = pl ? cJSON_GetObjectItem(pl, "grant") : NULL;
    if (!g) {
        cJSON_Delete(root);
        return false;
    }
    const cJSON *j;
    uint32_t freq = (j = cJSON_GetObjectItem(g, "frequency_hz")) && cJSON_IsNumber(j) ? (uint32_t)j->valuedouble : 0;
    uint32_t gid = (j = cJSON_GetObjectItem(g, "group_id")) && cJSON_IsNumber(j) ? (uint32_t)j->valuedouble : 0;
    char sys[32] = "", label[40] = "", serial[48] = "";
    if ((j = cJSON_GetObjectItem(g, "system")) && cJSON_IsString(j)) strlcpy(sys, j->valuestring, sizeof(sys));
    const cJSON *tg = cJSON_GetObjectItem(pl, "talkgroup");
    if (tg && (j = cJSON_GetObjectItem(tg, "alpha_tag")) && cJSON_IsString(j)) strlcpy(label, j->valuestring, sizeof(label));
    if ((j = cJSON_GetObjectItem(pl, "device_serial")) && cJSON_IsString(j)) strlcpy(serial, j->valuestring, sizeof(serial));
    uint32_t dur = (j = cJSON_GetObjectItem(pl, "duration_ms")) && cJSON_IsNumber(j) ? (uint32_t)j->valuedouble : 0;
    cJSON_Delete(root);

    if (!label[0]) {
        if (gid) snprintf(label, sizeof(label), "TG %lu", (unsigned long)gid);
        else if (freq) fmt_freq_label(freq, label, sizeof(label));
        else strlcpy(label, sys[0] ? sys : "?", sizeof(label));
    }
    if (!serial[0]) snprintf(serial, sizeof(serial), "%lu", (unsigned long)freq);

    time_t now = time(NULL);
    LOCK();
    ab_channel_t *c = channel_upsert(serial, label, freq);
    if (start) {
        if (c) {
            c->on_air = true;
            c->last = now;
            c->calls++;
        }
        s_ab->calls++;
        strlcpy(s_ab->system, sys, sizeof(s_ab->system));
        s_ab->on_air_since_ms = now_ms();
        memmove(&s_ab->log[1], &s_ab->log[0], sizeof(ab_log_t) * (AB_LOG_LEN - 1));
        ab_log_t *e = &s_ab->log[0];
        memset(e, 0, sizeof(*e));
        strlcpy(e->label, label, sizeof(e->label));
        e->freq_hz = freq;
        e->start = now;
        if (s_ab->log_n < AB_LOG_LEN) s_ab->log_n++;
    } else {
        if (c) {
            c->on_air = false;
            c->last = now;
        }
        for (int i = 0; i < s_ab->log_n; i++) {
            ab_log_t *e = &s_ab->log[i];
            if (e->dur_ms == 0 && e->freq_hz == freq && strcmp(e->label, label) == 0) {
                e->dur_ms = dur ? dur : (uint32_t)((now - e->start) * 1000) + 1;
                break;
            }
        }
    }
    refresh_on_air();
    UNLOCK();
    return true;
}

static void event_task(void *arg)
{
    uint8_t *buf = malloc(RX_BUF);
    char *line = heap_caps_malloc(EV_LINE_MAX, MALLOC_CAP_SPIRAM);
    char *data = heap_caps_malloc(EV_LINE_MAX, MALLOC_CAP_SPIRAM);
    int backoff = 2000;

    for (;;) {
        s_event_restart = false;
        const app_settings_t *cfg = settings_get();
        if (!cfg->ab_host[0]) {
            set_link(false, AB_OFF, "No GopherTrunk server set");
            wait_restart(&s_event_restart, 1000);
            continue;
        }
        if (!wifi_is_connected()) {
            set_link(false, AB_WAIT_WIFI, "");
            wifi_wait_connected(pdMS_TO_TICKS(1000));
            continue;
        }
        char host[64];
        strlcpy(host, cfg->ab_host, sizeof(host));
        uint16_t port = cfg->ab_port;
        int sock = http_get(false, host, port, "/api/v1/events");
        if (sock < 0) {
            wait_restart(&s_event_restart, backoff);
            backoff = backoff < 30000 ? backoff * 2 : 30000;
            continue;
        }
        char detail[96];
        snprintf(detail, sizeof(detail), "%s:%u", host, port);
        set_link(false, AB_CONNECTED, detail);
        ESP_LOGI(TAG, "events from %s", detail);
        backoff = 2000;

        bool in_hdr = true;
        int len = 0, dlen = 0;
        int kind = 0;            /* 0 other, 1 call.start, 2 call.end */
        while (!s_event_restart) {
            int n = recv(sock, buf, RX_BUF, 0);
            if (n == 0) {
                set_link(false, AB_ERROR, "Server closed the event stream");
                break;
            }
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    if (!wifi_is_connected()) break;
                    continue;
                }
                snprintf(detail, sizeof(detail), "recv: %s", strerror(errno));
                set_link(false, AB_ERROR, detail);
                break;
            }
            for (int i = 0; i < n; i++) {
                char c = (char)buf[i];
                if (c != '\n') {
                    if (len < EV_LINE_MAX - 1) line[len++] = c;
                    continue;
                }
                while (len && line[len - 1] == '\r') len--;
                line[len] = 0;
                if (in_hdr) {
                    if (len == 0) in_hdr = false;
                    else if (strncmp(line, "HTTP/", 5) == 0 && !strstr(line, " 200")) {
                        snprintf(detail, sizeof(detail), "server: %.60s", line);
                        set_link(false, AB_ERROR, detail);
                        s_event_restart = true;
                    }
                } else if (len == 0) {
                    /* end of one SSE record */
                    if (kind && dlen) {
                        data[dlen] = 0;
                        handle_call(kind == 1, data);
                    }
                    kind = 0;
                    dlen = 0;
                } else if (strncmp(line, "event:", 6) == 0) {
                    const char *k = line + 6;
                    while (*k == ' ') k++;
                    kind = strcmp(k, "call.start") == 0 ? 1 : strcmp(k, "call.end") == 0 ? 2 : 0;
                    dlen = 0;
                } else if (kind && strncmp(line, "data:", 5) == 0) {
                    const char *d = line + 5;
                    if (*d == ' ') d++;
                    size_t dl = strlen(d);
                    if (dlen + dl < EV_LINE_MAX - 1) {
                        memcpy(data + dlen, d, dl);
                        dlen += dl;
                    }
                }
                len = 0;
            }
        }
        close(sock);
        /* nothing is known to be on air any more */
        LOCK();
        for (int i = 0; i < s_ab->ch_n; i++) s_ab->ch[i].on_air = false;
        refresh_on_air();
        UNLOCK();
        wait_restart(&s_event_restart, 2000);
    }
}

/* ------------------------------------------------------------------------- */

static void parser_selftest(void)
{
    static const char start[] =
        "{\"kind\":\"call.start\",\"timestamp\":\"2026-11-04T17:00:00Z\",\"payload\":{\"grant\":{\"system\":\"scanner\","
        "\"protocol\":\"am-conv\",\"group_id\":0,\"source_id\":0,\"frequency_hz\":118700000},\"talkgroup\":{\"id\":0,"
        "\"alpha_tag\":\"Tower\",\"scan\":true,\"stream\":true,\"record\":true,\"mute\":false,\"discovered\":false},"
        "\"device_serial\":\"wb:00000008:conv:118700000\",\"started_at\":\"2026-11-04T17:00:00Z\"}}";
    static const char end[] =
        "{\"kind\":\"call.end\",\"payload\":{\"grant\":{\"system\":\"scanner\",\"frequency_hz\":118700000},"
        "\"talkgroup\":{\"alpha_tag\":\"Tower\"},\"device_serial\":\"wb:00000008:conv:118700000\",\"duration_ms\":4200}}";
    bool ok = handle_call(true, start);
    ok = ok && s_ab->on_air && strcmp(s_ab->label, "Tower") == 0 && s_ab->freq_hz == 118700000 && s_ab->ch_n == 1;
    ok = ok && handle_call(false, end) && !s_ab->on_air && s_ab->log_n == 1 && s_ab->log[0].dur_ms == 4200;
    if (ok) ESP_LOGI(TAG, "event parser self-test: passed");
    else ESP_LOGE(TAG, "event parser self-test: FAILED");
    memset(s_ab, 0, sizeof(*s_ab));
}

/* ------------------------------------------------------------------------- */
/* Tuning (GopherTrunk scanner manual tune)                                  */
/* ------------------------------------------------------------------------- */

typedef enum { CMD_TUNE, CMD_SCAN, CMD_SYNC } tune_kind_t;

typedef struct {
    tune_kind_t kind;
    uint32_t hz;
    char label[40];
} tune_cmd_t;

/* A channel on GopherTrunk's conventional scan list, identified by frequency + label.
 * Indexes are never remembered: GopherTrunk renumbers temporary channels when one is
 * removed, so a channel is always looked up again before it is touched. */
typedef struct {
    uint32_t hz;
    char label[40];
} chan_ref_t;

typedef struct {
    int idx;
    uint32_t hz;
    char label[40];
    bool locked;
} gt_chan_t;

#define MAX_REF 16
#define MAX_GT  96
#define BIG_N   (48 * 1024)
#define NVS_AB  "airband"

static QueueHandle_t s_tune_q;
static char *s_big;                  /* PSRAM buffer for GET /api/v1/scanner */
static gt_chan_t *s_gt;              /* PSRAM: GopherTrunk's scan list, last fetched */
static int s_gt_n;

/* What the display has done to GopherTrunk, kept in NVS so a reboot can carry on
 * (or clean up) instead of leaking channels:
 *   s_own    temporary channels the display added (guards and the tuned channel)
 *   s_locked channels the display locked out while holding a frequency
 *   s_tuned  the frequency being held; hz 0 = scanning */
static chan_ref_t s_own[MAX_REF], s_locked[MAX_REF], s_tuned;
static int s_own_n, s_locked_n;
static bool s_synced;

static void state_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_AB, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, "t_idx");           /* 1.3.0 keys */
    nvs_set_blob(h, "own", s_own, s_own_n * sizeof(chan_ref_t));
    nvs_set_blob(h, "locked", s_locked, s_locked_n * sizeof(chan_ref_t));
    nvs_set_u32(h, "t_hz", s_tuned.hz);
    nvs_set_str(h, "t_lab", s_tuned.label);
    nvs_commit(h);
    nvs_close(h);
}

static void state_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_AB, NVS_READONLY, &h) != ESP_OK) return;
    size_t n = sizeof(s_tuned.label);
    if (nvs_get_u32(h, "t_hz", &s_tuned.hz) != ESP_OK || nvs_get_str(h, "t_lab", s_tuned.label, &n) != ESP_OK) {
        memset(&s_tuned, 0, sizeof(s_tuned));
    }
    n = sizeof(s_own);
    if (nvs_get_blob(h, "own", s_own, &n) == ESP_OK) {
        s_own_n = n / sizeof(chan_ref_t);
    } else if (s_tuned.hz) {
        s_own[0] = s_tuned;              /* 1.3.0 kept just the one manual channel */
        s_own_n = 1;
    }
    n = sizeof(s_locked);
    if (nvs_get_blob(h, "locked", s_locked, &n) == ESP_OK) s_locked_n = n / sizeof(chan_ref_t);
    nvs_close(h);
}

static bool ref_eq(const chan_ref_t *a, uint32_t hz, const char *label)
{
    return a->hz == hz && strcmp(a->label, label) == 0;
}

static int ref_find(const chan_ref_t *list, int n, uint32_t hz, const char *label)
{
    for (int i = 0; i < n; i++) {
        if (ref_eq(&list[i], hz, label)) return i;
    }
    return -1;
}

static void ref_add(chan_ref_t *list, int *n, const chan_ref_t *r)
{
    if (ref_find(list, *n, r->hz, r->label) >= 0 || *n >= MAX_REF) return;
    list[(*n)++] = *r;
}

static void ref_del(chan_ref_t *list, int *n, int i)
{
    memmove(&list[i], &list[i + 1], (*n - i - 1) * sizeof(chan_ref_t));
    (*n)--;
}

uint32_t airband_parse_freq(const char *t)
{
    while (*t == ' ') t++;
    char *end;
    double v = strtod(t, &end);
    if (end == t || v <= 0) return 0;
    double hz;
    if (v < 2000) hz = v * 1e6;          /* MHz */
    else if (v < 2e6) hz = v * 1e3;      /* kHz */
    else hz = v;                         /* Hz */
    /* 1 kHz resolution: 8.33 kHz carriers are not on a 5 kHz grid */
    uint32_t r = (uint32_t)((hz + 500) / 1000) * 1000;
    return (r >= 25000000 && r <= 1300000000) ? r : 0;
}

int airband_parse_presets(const char *text, ab_preset_t *out, int max)
{
    int n = 0;
    char item[64];
    const char *p = text;
    while (*p && n < max) {
        size_t k = strcspn(p, ";,\n");
        size_t m = k < sizeof(item) - 1 ? k : sizeof(item) - 1;
        memcpy(item, p, m);
        item[m] = 0;
        p += k;
        if (*p) p++;
        char *s = item;
        while (*s == ' ') s++;
        char *e = s + strlen(s);
        while (e > s && e[-1] == ' ') *--e = 0;
        if (!*s) continue;
        /* the last word is the frequency */
        char *sp = strrchr(s, ' ');
        uint32_t hz = airband_parse_freq(sp ? sp + 1 : s);
        if (!hz) continue;
        if (sp) {
            *sp = 0;
            while (sp > s && sp[-1] == ' ') *--sp = 0;
            strlcpy(out[n].label, s, sizeof(out[n].label));
        } else {
            snprintf(out[n].label, sizeof(out[n].label), "%lu.%03lu", (unsigned long)(hz / 1000000),
                     (unsigned long)(hz / 1000 % 1000));
        }
        out[n].hz = hz;
        n++;
    }
    return n;
}

static void tune_status(bool busy, bool err, const char *fmt, ...)
{
    char b[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    LOCK();
    s_ab->tune_busy = busy;
    s_ab->tune_error = err;
    strlcpy(s_ab->tune_status, b, sizeof(s_ab->tune_status));
    UNLOCK();
}

/* One-shot HTTP request; returns the status code (or -1) and the body in `resp` */
static int http_request(const char *method, const char *path, const char *body, char *resp, size_t resp_n)
{
    const app_settings_t *cfg = settings_get();
    char host[64], port_s[8];
    strlcpy(host, cfg->ab_host, sizeof(host));
    snprintf(port_s, sizeof(port_s), "%u", cfg->ab_port);
    resp[0] = 0;
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, port_s, &hints, &res) != 0 || !res) return -1;
    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        freeaddrinfo(res);
        return -1;
    }
    struct timeval tv = {.tv_sec = 5};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int r = connect(sock, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (r < 0) {
        close(sock);
        return -1;
    }
    char auth[128] = "";
    if (cfg->ab_token[0]) snprintf(auth, sizeof(auth), "Authorization: Bearer %s\r\n", cfg->ab_token);
    char req[512];
    size_t bl = body ? strlen(body) : 0;
    int n = snprintf(req, sizeof(req),
                     "%s %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: adsb-display\r\n%s"
                     "Content-Type: application/json\r\nContent-Length: %u\r\n\r\n%s",
                     method, path, host, auth, (unsigned)bl, body ? body : "");
    if (n >= (int)sizeof(req) || send(sock, req, n, 0) != n) {
        close(sock);
        return -1;
    }
    size_t got = 0;
    while (got < resp_n - 1) {
        int k = recv(sock, resp + got, resp_n - 1 - got, 0);
        if (k <= 0) break;
        got += k;
    }
    close(sock);
    resp[got] = 0;
    int status = -1;
    if (strncmp(resp, "HTTP/", 5) == 0) {
        const char *sp = strchr(resp, ' ');
        if (sp) status = atoi(sp + 1);
    }
    char *b = strstr(resp, "\r\n\r\n");
    if (b) memmove(resp, b + 4, strlen(b + 4) + 1);
    else resp[0] = 0;
    return status;
}

/* {"error":"..."} -> message, for the status line */
static void api_error(const char *body, char *out, size_t n)
{
    out[0] = 0;
    cJSON *j = cJSON_Parse(body);
    const cJSON *e = j ? cJSON_GetObjectItem(j, "error") : NULL;
    if (e && cJSON_IsString(e)) strlcpy(out, e->valuestring, n);
    cJSON_Delete(j);
}

static void fmt_mhz(uint32_t hz, char *out, size_t n)
{
    snprintf(out, n, "%lu.%03lu", (unsigned long)(hz / 1000000), (unsigned long)(hz / 1000 % 1000));
}

/* label as GopherTrunk will store it: JSON-safe, and never empty */
static void clean_label(const char *in, uint32_t hz, char *out, size_t n)
{
    size_t o = 0;
    for (const char *c = in ? in : ""; *c && o < n - 1; c++) {
        if (*c != '"' && *c != 0x5C && (unsigned char)*c >= 0x20) out[o++] = *c;
    }
    out[o] = 0;
    if (!o) fmt_mhz(hz, out, n);
}

/* "121500,118100" (kHz) */
bool airband_guard_has(const char *list, uint32_t hz)
{
    const char *p = list;
    while (p && *p) {
        if ((uint32_t)strtoul(p, NULL, 10) * 1000 == hz) return true;
        p = strchr(p, ',');
        if (p) p++;
    }
    return false;
}

void airband_guard_toggle(char *list, size_t n, uint32_t hz)
{
    char out[160] = "";
    bool had = false;
    const char *p = list;
    while (p && *p) {
        uint32_t k = strtoul(p, NULL, 10);
        if (k * 1000 == hz) had = true;
        else if (k) snprintf(out + strlen(out), sizeof(out) - strlen(out), "%s%lu", out[0] ? "," : "", (unsigned long)k);
        p = strchr(p, ',');
        if (p) p++;
    }
    if (!had) snprintf(out + strlen(out), sizeof(out) - strlen(out), "%s%lu", out[0] ? "," : "", (unsigned long)(hz / 1000));
    strlcpy(list, out, n);
}

/* The channels the display wants on the scan list: guarded presets, then the tuned
 * frequency (last, so it is the one GopherTrunk dwells on). Returns the count and how
 * many of them are guards. */
static int wanted(chan_ref_t *out, int *guards)
{
    const app_settings_t *cfg = settings_get();
    static ab_preset_t pre[12];          /* tune task only */
    int pn = airband_parse_presets(cfg->ab_presets, pre, 12);
    int n = 0;
    bool tuned_is_guard = false;
    for (int i = 0; i < pn && n < MAX_REF - 1; i++) {
        if (!airband_guard_has(cfg->ab_guard, pre[i].hz)) continue;
        chan_ref_t r = {.hz = pre[i].hz};
        clean_label(pre[i].label, pre[i].hz, r.label, sizeof(r.label));
        if (ref_find(out, n, r.hz, r.label) >= 0) continue;
        if (s_tuned.hz == r.hz) {
            tuned_is_guard = true;
            s_tuned = r;                 /* hold on the guard's own channel */
        }
        out[n++] = r;
    }
    *guards = n;
    if (s_tuned.hz && !tuned_is_guard) out[n++] = s_tuned;
    return n;
}

/* GET /api/v1/scanner into s_gt; returns the HTTP status */
static int gt_fetch(void)
{
    s_gt_n = 0;
    int st = http_request("GET", "/api/v1/scanner", NULL, s_big, BIG_N);
    if (st != 200) return st;
    cJSON *root = cJSON_Parse(s_big);
    if (!root) return -1;
    const cJSON *conv = cJSON_GetObjectItem(root, "conventional");
    const cJSON *chans = conv ? cJSON_GetObjectItem(conv, "channels") : NULL;
    const cJSON *c;
    cJSON_ArrayForEach(c, chans) {
        const cJSON *ix = cJSON_GetObjectItem(c, "index");
        const cJSON *fq = cJSON_GetObjectItem(c, "frequency_hz");
        const cJSON *lb = cJSON_GetObjectItem(c, "label");
        if (!cJSON_IsNumber(ix) || !cJSON_IsNumber(fq) || !cJSON_IsString(lb) || s_gt_n >= MAX_GT) continue;
        gt_chan_t *g = &s_gt[s_gt_n++];
        g->idx = ix->valueint;
        g->hz = (uint32_t)fq->valuedouble;
        strlcpy(g->label, lb->valuestring, sizeof(g->label));
        g->locked = cJSON_IsTrue(cJSON_GetObjectItem(c, "locked_out"));
    }
    cJSON_Delete(root);
    return 200;
}

/* the matching channel with the highest index (the most recently added), or NULL */
static gt_chan_t *gt_find(const chan_ref_t *r)
{
    gt_chan_t *best = NULL;
    for (int i = 0; i < s_gt_n; i++) {
        if (ref_eq(r, s_gt[i].hz, s_gt[i].label) && (!best || s_gt[i].idx > best->idx)) best = &s_gt[i];
    }
    return best;
}

static int gt_post(const char *fmt, int idx)
{
    char path[80], resp[512];
    snprintf(path, sizeof(path), fmt, idx);
    return http_request("POST", path, "", resp, sizeof(resp));
}

/* Make GopherTrunk's scan list match what the display wants:
 *  - remove temporary channels the display added and no longer wants
 *  - add the guarded presets and the tuned frequency
 *  - holding: lock out every other channel, so the scanner only visits the tuned
 *    frequency and the guards; scanning: unlock what the display locked, resume
 * Returns 200, or the HTTP status (-1: unreachable) of the step that failed, with the
 * GopherTrunk error message in err. */
static int sync_scanner(bool resume, int *guards, char *err, size_t errn)
{
    chan_ref_t want[MAX_REF];
    int wn = wanted(want, guards);
    char body[160], resp[1024];
    err[0] = 0;

    int st = gt_fetch();
    if (st != 200) return st;

    for (int i = 0; i < s_own_n;) {
        if (ref_find(want, wn, s_own[i].hz, s_own[i].label) >= 0) {
            i++;
            continue;
        }
        gt_chan_t *g = gt_find(&s_own[i]);
        if (g) {
            char path[64];
            snprintf(path, sizeof(path), "/api/v1/scanner/manual_tune/%d", g->idx);
            st = http_request("DELETE", path, NULL, resp, sizeof(resp));
            if (st != 200 && st != 404) {      /* 404: a config channel, i.e. not ours after all */
                api_error(resp, err, errn);
                return st;
            }
            if ((st = gt_fetch()) != 200) return st;
        }
        ref_del(s_own, &s_own_n, i);
        state_save();
    }

    bool added = false;
    for (int i = 0; i < wn; i++) {
        if (gt_find(&want[i])) {
            ref_add(s_own, &s_own_n, &want[i]);   /* e.g. left from before a restart */
            continue;
        }
        snprintf(body, sizeof(body), "{\"frequency_hz\":%lu,\"label\":\"%s\",\"mode\":\"am\"}",
                 (unsigned long)want[i].hz, want[i].label);
        st = http_request("POST", "/api/v1/scanner/manual_tune", body, resp, sizeof(resp));
        if (st != 200) {
            api_error(resp, err, errn);
            return st;
        }
        ref_add(s_own, &s_own_n, &want[i]);
        state_save();
        added = true;
    }
    if (added && (st = gt_fetch()) != 200) return st;

    if (s_tuned.hz) {
        for (int i = 0; i < s_gt_n; i++) {
            const gt_chan_t *g = &s_gt[i];
            chan_ref_t r = {.hz = g->hz};
            strlcpy(r.label, g->label, sizeof(r.label));
            if (g->locked || ref_find(want, wn, r.hz, r.label) >= 0) continue;
            if ((st = gt_post("/api/v1/scanner/conventional/%d/lockout", g->idx)) != 200) return st;
            ref_add(s_locked, &s_locked_n, &r);
            state_save();
        }
    }
    for (int i = 0; i < s_locked_n;) {
        if (s_tuned.hz && ref_find(want, wn, s_locked[i].hz, s_locked[i].label) < 0) {
            i++;
            continue;
        }
        gt_chan_t *g = gt_find(&s_locked[i]);
        if (g && g->locked && (st = gt_post("/api/v1/scanner/conventional/%d/unlockout", g->idx)) != 200) return st;
        ref_del(s_locked, &s_locked_n, i);
        state_save();
    }

    if (s_tuned.hz) {
        gt_chan_t *g = gt_find(&s_tuned);
        if (g && (st = gt_post("/api/v1/scanner/conventional/%d/dwell", g->idx)) != 200) return st;
    }
    if (resume && (st = gt_post("/api/v1/scanner/conventional/resume", 0)) != 200) return st;
    return 200;
}

static void report(int st, int guards, const char *err, bool after_boot)
{
    char mhz[16];
    fmt_mhz(s_tuned.hz, mhz, sizeof(mhz));
    LOCK();
    s_ab->tuned = st == 200 && s_tuned.hz;
    s_ab->tuned_hz = s_tuned.hz;
    strlcpy(s_ab->tuned_label, s_tuned.label, sizeof(s_ab->tuned_label));
    s_ab->guards = guards;
    UNLOCK();
    if (st == 200) {
        char g[40] = "";
        if (guards) snprintf(g, sizeof(g), ", guarding %d", guards);
        if (s_tuned.hz) tune_status(false, false, "Holding %s MHz%s%s", mhz, g, after_boot ? " (from before restart)" : "");
        else tune_status(false, false, "Scanning%s", g);
    } else if (st == 401 || st == 403) {
        tune_status(false, true, "GopherTrunk refused: set the API token in Settings");
    } else if (st == 503) {
        tune_status(false, true, "GopherTrunk has no scan-list scanner to tune");
    } else if (st < 0) {
        tune_status(false, true, "Can't reach GopherTrunk");
    } else {
        tune_status(false, true, "GopherTrunk said %d %s", st, err);
    }
}

static void tune_task(void *arg)
{
    tune_cmd_t cmd;
    char err[80];
    for (;;) {
        /* after boot: pick up (or clean up) what the display left on GopherTrunk; retry every 15 s */
        if (!s_synced && settings_get()->ab_host[0] && wifi_is_connected()) {
            int guards = 0;
            chan_ref_t want[MAX_REF];
            if (!s_own_n && !s_locked_n && !wanted(want, &guards)) {
                s_synced = true;             /* nothing to do: don't touch GopherTrunk */
            } else {
                int st = sync_scanner(false, &guards, err, sizeof(err));
                s_synced = st == 200 || (st > 0 && st != 503 && st < 500);
                if (s_synced) report(st, guards, err, true);
            }
        }
        if (xQueueReceive(s_tune_q, &cmd, s_synced ? portMAX_DELAY : pdMS_TO_TICKS(15000)) != pdTRUE) continue;
        if (!settings_get()->ab_host[0]) {
            tune_status(false, true, "Set the GopherTrunk server in Settings");
            continue;
        }
        if (!wifi_is_connected()) {
            tune_status(false, true, "No Wi-Fi");
            continue;
        }
        char mhz[16];
        if (cmd.kind == CMD_TUNE) {
            s_tuned.hz = cmd.hz;
            clean_label(cmd.label, cmd.hz, s_tuned.label, sizeof(s_tuned.label));
            fmt_mhz(cmd.hz, mhz, sizeof(mhz));
            tune_status(true, false, "Tuning %s...", mhz);
        } else if (cmd.kind == CMD_SCAN) {
            memset(&s_tuned, 0, sizeof(s_tuned));
            tune_status(true, false, "Resuming scan...");
        } else {
            tune_status(true, false, "Updating guard channels...");
        }
        state_save();
        int guards = 0;
        int st = sync_scanner(cmd.kind == CMD_SCAN, &guards, err, sizeof(err));
        s_synced = true;
        report(st, guards, err, false);
    }
}

static void tune_send(tune_kind_t kind, uint32_t hz, const char *label)
{
    if (!s_tune_q) return;
    tune_cmd_t c = {.kind = kind, .hz = hz};
    strlcpy(c.label, label ? label : "", sizeof(c.label));
    xQueueSend(s_tune_q, &c, 0);
}

void airband_tune(uint32_t hz, const char *label)
{
    if (hz) tune_send(CMD_TUNE, hz, label);
}

void airband_resume_scan(void)
{
    tune_send(CMD_SCAN, 0, NULL);
}

void airband_sync_guards(void)
{
    tune_send(CMD_SYNC, 0, NULL);
}

void airband_start(void)
{
    s_ab = heap_caps_calloc(1, sizeof(ab_state_t), MALLOC_CAP_SPIRAM);
    s_lock = xSemaphoreCreateMutex();
    s_sb = xStreamBufferCreateWithCaps(SB_BYTES, 1, MALLOC_CAP_SPIRAM);
    assert(s_ab && s_lock && s_sb);
    parser_selftest();

    xTaskCreatePinnedToCore(audio_task, "ab_audio", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(player_task, "ab_play", 4096, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(event_task, "ab_events", 6144, NULL, 4, NULL, 1);
    s_tune_q = xQueueCreate(4, sizeof(tune_cmd_t));
    s_big = heap_caps_malloc(BIG_N, MALLOC_CAP_SPIRAM);
    s_gt = heap_caps_calloc(MAX_GT, sizeof(gt_chan_t), MALLOC_CAP_SPIRAM);
    assert(s_big && s_gt);
    state_load();
    xTaskCreatePinnedToCore(tune_task, "ab_tune", 8192, NULL, 4, NULL, 1);
}
