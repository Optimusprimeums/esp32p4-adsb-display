#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Online enrichment from api.adsbdb.com:
 *   registration, type, manufacturer, owner/operator, airline, origin/destination
 *   and an aircraft photo thumbnail (JPEG) for the selected aircraft.
 */
void lookup_start(void);

/* Aircraft the user is looking at: looked up first, and its photo is fetched */
void lookup_set_priority(uint32_t addr);

/* If a new photo is ready, hands ownership of the JPEG buffer to the caller
 * (free() it when done). Returns NULL if nothing new. */
uint8_t *lookup_photo_take(uint32_t *addr, size_t *len);

typedef struct {
    uint32_t requests, ok, not_found, errors;
    char last_error[48];
} lookup_stats_t;

void lookup_get_stats(lookup_stats_t *out);
