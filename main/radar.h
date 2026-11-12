#pragma once

/* Weather radar overlay for the map, from RainViewer's public API
 * (https://www.rainviewer.com/api.html). Built only with CONFIG_ADSB_WX_VIEW.
 *
 * The UI says which part of the world the map shows (radar_set_view); a
 * background task fetches the radar tiles for it and reprojects them into an
 * ARGB8888 image the size of the map. The UI picks the finished image up from
 * the LVGL task (radar_take), so the image it draws is never written to. */

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "lvgl.h"

typedef struct {
    double clat, clon;      /* map centre */
    float px_per_nm;        /* map scale */
    int32_t w, h;           /* map size, px */
} radar_view_t;

typedef struct {
    bool ok;                /* an image is available */
    time_t frame_time;      /* radar observation time of the image */
    char status[64];        /* "" or what went wrong */
} radar_info_t;

void radar_start(void);
/* turn fetching on/off (off frees nothing, it just stops network traffic) */
void radar_enable(bool on);
/* the map moved or zoomed; the image is re-rendered once it settles */
void radar_set_view(const radar_view_t *v);
/* LVGL task: the latest finished image, or NULL. *view is the view it was
 * rendered for. The image stays valid until the next call. */
const lv_image_dsc_t *radar_take(radar_view_t *view);
void radar_get_info(radar_info_t *out);
