#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FEED_IDLE = 0,
    FEED_WAIT_WIFI,
    FEED_RESOLVING,
    FEED_CONNECTING,
    FEED_CONNECTED,
    FEED_ERROR,
} feed_state_t;

typedef struct {
    feed_state_t state;
    char detail[64];          /* last error / info */
    uint32_t bytes_rx;
    uint32_t frames_rx;
    uint32_t connects;
    uint32_t connected_since_ms;
} feed_status_t;

void adsb_client_start(void);
/* Drop the connection and reconnect with current settings */
void adsb_client_restart(void);
void adsb_client_status(feed_status_t *out);
const char *feed_state_str(feed_state_t s);
