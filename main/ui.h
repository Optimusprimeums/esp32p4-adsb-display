#pragma once

#include "lvgl.h"

/* Build the UI on the given display. Call from the LVGL task. */
void ui_init(lv_display_t *disp);
