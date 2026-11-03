#include "adsb_client.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "aircraft.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "settings.h"
#include "wifi.h"

static const char *TAG = "feed";

#define RX_BUF_SIZE       4096
#define IDLE_TIMEOUT_MS   60000   /* reconnect if the server goes silent */
#define LINE_MAX_LEN      512

static feed_status_t s_status;
static volatile bool s_restart;

const char *feed_state_str(feed_state_t s)
{
    switch (s) {
    case FEED_IDLE:       return "Idle";
    case FEED_WAIT_WIFI:  return "Waiting for Wi-Fi";
    case FEED_RESOLVING:  return "Resolving";
    case FEED_CONNECTING: return "Connecting";
    case FEED_CONNECTED:  return "Connected";
    case FEED_ERROR:      return "Error";
    }
    return "?";
}

void adsb_client_status(feed_status_t *out)
{
    *out = s_status;
}

void adsb_client_restart(void)
{
    s_restart = true;
}

static void set_state(feed_state_t st, const char *detail)
{
    s_status.state = st;
    strlcpy(s_status.detail, detail ? detail : "", sizeof(s_status.detail));
}

/* ------------------------------------------------------------------------- */
/* Beast binary                                                              */
/* ------------------------------------------------------------------------- */

typedef struct {
    enum { B_SYNC, B_TYPE, B_DATA } st;
    bool esc;
    uint8_t type;
    uint8_t buf[6 + 1 + 14];
    int need, have;
} beast_t;

static int beast_len(uint8_t type)
{
    switch (type) {
    case '1': return 6 + 1 + 2;
    case '2': return 6 + 1 + 7;
    case '3': return 6 + 1 + 14;
    default:  return -1;
    }
}

static void beast_frame(beast_t *b)
{
    uint64_t ts = 0;
    for (int i = 0; i < 6; i++) ts = (ts << 8) | b->buf[i];
    uint8_t sig = b->buf[6];
    float rssi = sig ? 20.0f * log10f(sig / 255.0f) : NAN;
    s_status.frames_rx++;
    if (b->type == '2') aircraft_ingest_frame(&b->buf[7], 7, rssi, ts / 12);
    else if (b->type == '3') aircraft_ingest_frame(&b->buf[7], 14, rssi, ts / 12);
    /* '1' = Mode A/C, not used */
}

static void beast_feed(beast_t *b, const uint8_t *data, int n)
{
    for (int i = 0; i < n; i++) {
        uint8_t c = data[i];
        switch (b->st) {
        case B_SYNC:
            if (c == 0x1A) b->st = B_TYPE;
            break;
        case B_TYPE:
            b->need = beast_len(c);
            if (b->need < 0) {
                b->st = B_SYNC;   /* 1A 1A is an escaped data byte, not a frame start */
                break;
            }
            b->type = c;
            b->have = 0;
            b->esc = false;
            b->st = B_DATA;
            break;
        case B_DATA:
            if (b->esc) {
                b->esc = false;
                if (c != 0x1A) {
                    /* unescaped 0x1A: that was the start of a new frame */
                    b->need = beast_len(c);
                    if (b->need < 0) { b->st = B_SYNC; break; }
                    b->type = c;
                    b->have = 0;
                    break;
                }
            } else if (c == 0x1A) {
                b->esc = true;
                break;
            }
            b->buf[b->have++] = c;
            if (b->have == b->need) {
                beast_frame(b);
                b->st = B_SYNC;
            }
            break;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Line-based protocols (AVR, SBS)                                           */
/* ------------------------------------------------------------------------- */

typedef struct {
    char line[LINE_MAX_LEN];
    int len;
    bool overflow;
} liner_t;

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void avr_line(char *s)
{
    /* *8D...;   @<12 hex ts>8D...;   %<12 ts>...;   <<12 ts><2 sig>8D...; */
    float rssi = NAN;
    char *p = s;
    while (*p == ' ' || *p == '\t') p++;
    char kind = *p++;
    if (kind == '@' || kind == '%') {
        if (strlen(p) < 12) return;
        p += 12;
    } else if (kind == '<') {
        if (strlen(p) < 14) return;
        int hi = hexval(p[12]), lo = hexval(p[13]);
        if (hi >= 0 && lo >= 0) {
            int sig = hi * 16 + lo;
            if (sig) rssi = 20.0f * log10f(sig / 255.0f);
        }
        p += 14;
    } else if (kind != '*') {
        return;
    }
    uint8_t msg[14];
    int n = 0;
    while (p[0] && p[1] && p[0] != ';' && n < 14) {
        int hi = hexval(p[0]), lo = hexval(p[1]);
        if (hi < 0 || lo < 0) return;
        msg[n++] = (uint8_t)(hi * 16 + lo);
        p += 2;
    }
    if (n == 7 || n == 14) {
        s_status.frames_rx++;
        aircraft_ingest_frame(msg, n, rssi, 0);
    }
}

/* split CSV in place, keeping empty fields */
static int csv_split(char *s, char **f, int max)
{
    int n = 0;
    f[n++] = s;
    for (char *p = s; *p && n < max; p++) {
        if (*p == ',') {
            *p = 0;
            f[n++] = p + 1;
        }
    }
    return n;
}

static void sbs_line(char *s)
{
    char *f[24];
    int n = csv_split(s, f, 24);
    if (n < 11 || strcmp(f[0], "MSG") != 0) return;

    /* readsb marks non-ICAO (anonymous / TIS-B) addresses with a leading '~' */
    bool non_icao = f[4][0] == '~';
    uint32_t addr = strtoul(f[4] + (non_icao ? 1 : 0), NULL, 16);
    if (!addr) return;
    if (non_icao) addr |= 0x1000000;

    mm_t mm;
    memset(&mm, 0, sizeof(mm));
    mm.rssi_dbfs = NAN;
    mm.addr = addr;
    mm.crc_ok = true;

#define FIELD(i) (n > (i) && f[i][0])
    if (FIELD(10)) {
        strlcpy(mm.callsign, f[10], sizeof(mm.callsign));
        for (int i = strlen(mm.callsign) - 1; i >= 0 && mm.callsign[i] == ' '; i--) mm.callsign[i] = 0;
        mm.callsign_valid = mm.callsign[0] != 0;
    }
    if (FIELD(11)) { mm.alt_valid = true; mm.alt_ft = atoi(f[11]); mm.alt_src = ALT_SRC_BARO; }
    if (FIELD(12)) { mm.gs_valid = true; mm.gs_kt = strtof(f[12], NULL); }
    if (FIELD(13)) { mm.track_valid = true; mm.track_deg = strtof(f[13], NULL); }
    if (FIELD(16)) { mm.baro_rate_valid = true; mm.baro_rate_fpm = atoi(f[16]); }
    if (FIELD(17)) {
        mm.squawk_valid = true;
        mm.squawk = (uint16_t)strtoul(f[17], NULL, 16);  /* "7700" -> 0x7700 */
    }
    if (FIELD(18)) { mm.fs_valid = true; mm.alert = atoi(f[18]) != 0; }
    if (FIELD(20)) { mm.fs_valid = true; mm.spi = atoi(f[20]) != 0; }
    if (FIELD(21)) { mm.ground_valid = true; mm.on_ground = atoi(f[21]) != 0; }
    if (FIELD(19) && atoi(f[19]) != 0 && mm.squawk_valid) {
        mm.emergency_valid = true;
        mm.emergency = mm.squawk == 0x7500 ? EMERG_UNLAWFUL :
                       mm.squawk == 0x7600 ? EMERG_NORDO : EMERG_GENERAL;
    }
    bool pos = FIELD(14) && FIELD(15);
    double lat = pos ? strtod(f[14], NULL) : 0, lon = pos ? strtod(f[15], NULL) : 0;
#undef FIELD
    s_status.frames_rx++;
    aircraft_ingest_decoded(&mm, SRC_SBS, pos, lat, lon);
}

static void liner_feed(liner_t *l, const uint8_t *data, int n, void (*cb)(char *))
{
    for (int i = 0; i < n; i++) {
        char c = (char)data[i];
        if (c == '\n' || c == '\r') {
            if (l->len && !l->overflow) {
                l->line[l->len] = 0;
                cb(l->line);
            }
            l->len = 0;
            l->overflow = false;
        } else if (l->len < LINE_MAX_LEN - 1) {
            l->line[l->len++] = c;
            /* AVR frames end with ';' and some feeds omit newlines */
            if (c == ';' && cb == avr_line) {
                l->line[l->len] = 0;
                cb(l->line);
                l->len = 0;
            }
        } else {
            l->overflow = true;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Connection task                                                           */
/* ------------------------------------------------------------------------- */

static int connect_to(const char *host, uint16_t port)
{
    char port_s[8];
    snprintf(port_s, sizeof(port_s), "%u", port);
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *res = NULL;

    set_state(FEED_RESOLVING, host);
    int err = getaddrinfo(host, port_s, &hints, &res);
    if (err != 0 || !res) {
        set_state(FEED_ERROR, "DNS lookup failed");
        return -1;
    }

    char detail[96];
    snprintf(detail, sizeof(detail), "%s:%u", host, port);
    set_state(FEED_CONNECTING, detail);

    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        freeaddrinfo(res);
        set_state(FEED_ERROR, "socket() failed");
        return -1;
    }

    /* non-blocking connect with 5 s timeout */
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    int r = connect(sock, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (r < 0 && errno != EINPROGRESS) {
        snprintf(detail, sizeof(detail), "connect: %s", strerror(errno));
        set_state(FEED_ERROR, detail);
        close(sock);
        return -1;
    }
    if (r < 0) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(sock, &wfds);
        struct timeval tv = {.tv_sec = 5};
        r = select(sock + 1, NULL, &wfds, NULL, &tv);
        int so_err = 0;
        socklen_t len = sizeof(so_err);
        if (r <= 0 || getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_err, &len) < 0 || so_err) {
            snprintf(detail, sizeof(detail), "connect: %s", r == 0 ? "timeout" : strerror(so_err ? so_err : errno));
            set_state(FEED_ERROR, detail);
            close(sock);
            return -1;
        }
    }
    fcntl(sock, F_SETFL, flags);

    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    struct timeval rcv = {.tv_sec = 1};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv, sizeof(rcv));
    return sock;
}

static void feed_task(void *arg)
{
    uint8_t *buf = malloc(RX_BUF_SIZE);
    beast_t *beast = calloc(1, sizeof(beast_t));
    liner_t *liner = calloc(1, sizeof(liner_t));
    int backoff_ms = 1000;

    for (;;) {
        s_restart = false;
        if (!wifi_is_connected()) {
            set_state(FEED_WAIT_WIFI, wifi_state_str());
            wifi_wait_connected(pdMS_TO_TICKS(1000));
            continue;
        }

        const app_settings_t *cfg = settings_get();
        char host[64];
        strlcpy(host, cfg->host, sizeof(host));
        uint16_t port = cfg->port;
        feed_proto_t proto = cfg->proto;

        if (!host[0]) {
            set_state(FEED_ERROR, "No server configured");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int sock = connect_to(host, port);
        if (sock < 0) {
            ESP_LOGW(TAG, "%s", s_status.detail);
            for (int t = 0; t < backoff_ms && !s_restart; t += 100) vTaskDelay(pdMS_TO_TICKS(100));
            backoff_ms = backoff_ms < 15000 ? backoff_ms * 2 : 15000;
            continue;
        }

        char detail[96];
        snprintf(detail, sizeof(detail), "%s:%u (%s)", host, port, feed_proto_name(proto));
        set_state(FEED_CONNECTED, detail);
        ESP_LOGI(TAG, "connected to %s", detail);
        s_status.connects++;
        s_status.connected_since_ms = now_ms();
        aircraft_feed_event(true);
        backoff_ms = 1000;
        memset(beast, 0, sizeof(*beast));
        memset(liner, 0, sizeof(*liner));
        uint32_t last_rx = now_ms();

        while (!s_restart) {
            int n = recv(sock, buf, RX_BUF_SIZE, 0);
            if (n > 0) {
                last_rx = now_ms();
                s_status.bytes_rx += n;
                switch (proto) {
                case FEED_BEAST: beast_feed(beast, buf, n); break;
                case FEED_AVR:   liner_feed(liner, buf, n, avr_line); break;
                case FEED_SBS:   liner_feed(liner, buf, n, sbs_line); break;
                }
            } else if (n == 0) {
                set_state(FEED_ERROR, "Server closed connection");
                break;
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (now_ms() - last_rx > IDLE_TIMEOUT_MS) {
                    set_state(FEED_ERROR, "No data for 60 s");
                    break;
                }
                if (!wifi_is_connected()) break;
            } else {
                snprintf(detail, sizeof(detail), "recv: %s", strerror(errno));
                set_state(FEED_ERROR, detail);
                break;
            }
        }
        ESP_LOGW(TAG, "disconnected: %s", s_restart ? "restart requested" : s_status.detail);
        close(sock);
        aircraft_feed_event(false);
        if (!s_restart) vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void adsb_client_start(void)
{
    xTaskCreatePinnedToCore(feed_task, "feed", 6144, NULL, 5, NULL, 1);
}
