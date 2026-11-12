#pragma once

/*
 * Airband tab: live voice from a GopherTrunk scanner, played through the
 * board's ES8311 codec and speaker amplifier.
 *
 *   GET /api/v1/audio/stream  - open-ended WAV (16-bit LE mono PCM at the
 *                               daemon's audio.sample_rate), only while
 *                               "Listen" is on. ?device=<serial> narrows it to
 *                               one channel.
 *   GET /api/v1/events        - Server-Sent Events; call.start / call.end give
 *                               the channel label, frequency and duration.
 *
 * GopherTrunk only needs its HTTP API reachable on the LAN
 * (api.http_addr: "0.0.0.0:8080"); both endpoints are read-only GETs.
 */

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define AB_MAX_CHANNELS 16
#define AB_LOG_LEN      30

typedef enum {
    AB_OFF = 0,          /* no server configured / not listening */
    AB_WAIT_WIFI,
    AB_CONNECTING,
    AB_CONNECTED,
    AB_ERROR,
} ab_link_t;

typedef struct {
    char serial[48];     /* GopherTrunk device_serial, used as the audio filter */
    char label[40];
    uint32_t freq_hz;
    time_t last;         /* wall-clock time of the last transmission */
    uint32_t calls;
    bool on_air;
} ab_channel_t;

typedef struct {
    char label[40];
    uint32_t freq_hz;
    time_t start;
    uint32_t dur_ms;     /* 0 while still on air */
} ab_log_t;

typedef struct {
    /* links */
    ab_link_t audio_link, event_link;
    char audio_detail[64], event_detail[64];
    bool codec_ok;

    /* playback */
    uint32_t rate;           /* stream sample rate from the WAV header */
    bool playing;            /* audio is coming out of the speaker now */
    uint8_t level;           /* 0..100 peak meter */
    uint32_t buffered_ms;
    uint32_t bytes_rx, underruns;

    /* current transmission (from events) */
    bool on_air;
    char label[40];
    char system[32];
    uint32_t freq_hz;
    uint32_t on_air_since_ms;
    uint32_t calls;

    ab_channel_t ch[AB_MAX_CHANNELS];
    uint8_t ch_n;
    ab_log_t log[AB_LOG_LEN];   /* newest first */
    uint8_t log_n;

    /* tuning (GopherTrunk manual tune) */
    bool tune_busy;
    bool tuned;                 /* a manual channel is active */
    uint32_t tuned_hz;
    char tuned_label[40];
    int guards;                   /* guarded presets on the scan list */
    char tune_status[80];
    bool tune_error;
} ab_state_t;

/* Bring up the codec and start the client tasks (call after board_init) */
void airband_start(void);
/* Reconnect both links with the current settings (server, filter, listen) */
void airband_restart(void);
/* Apply settings->ab_volume / ab_mute immediately */
void airband_apply_volume(void);
/* Thread-safe copy (keep `out` static or in PSRAM) */
void airband_get(ab_state_t *out);
const char *ab_link_str(ab_link_t l);

/* Ask GopherTrunk to dwell on an AM frequency (replaces the previous manual
 * channel). Runs in the background; progress lands in tune_status. */
void airband_tune(uint32_t hz, const char *label);
/* Drop the manual channel and resume normal scanning */
void airband_resume_scan(void);
/* re-send the guarded presets to GopherTrunk (after the guard list or presets change) */
void airband_sync_guards(void);
/* guard list: preset frequencies in kHz, comma separated */
bool airband_guard_has(const char *list, uint32_t hz);
void airband_guard_toggle(char *list, size_t n, uint32_t hz);

/* Presets from settings->ab_presets ("Tower 118.700; Ground 121.900") */
typedef struct {
    char label[24];
    uint32_t hz;
} ab_preset_t;
int airband_parse_presets(const char *text, ab_preset_t *out, int max);
/* "118.100" / "118.1" / "118100" (kHz) -> Hz, 0 if invalid */
uint32_t airband_parse_freq(const char *text);
