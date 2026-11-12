#pragma once

#include "lvgl.h"

/* Build the UI on the given display. Call from the LVGL task. */
void ui_init(lv_display_t *disp);

/* Demo / screenshot automation: applied by the UI task within a second.
 * tab: 0..3 or -1 to keep; select: ICAO address, 0 = deselect, -1 = keep;
 * range_nm: 0 = keep. */
void ui_demo_request(int tab, int64_t select, int range_nm);
