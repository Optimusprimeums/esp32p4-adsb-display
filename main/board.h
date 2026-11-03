#pragma once

#include "esp_err.h"
#include "lvgl.h"

/* Guition JC8012P4A1 (10.1" 800x1280, JD9365 MIPI-DSI, GSL3680 touch) */
#define BOARD_LCD_H_RES        800
#define BOARD_LCD_V_RES        1280
#define BOARD_PIN_LCD_RST      27
#define BOARD_PIN_LCD_BL       23
#define BOARD_PIN_TP_SDA       7
#define BOARD_PIN_TP_SCL       8
#define BOARD_PIN_TP_RST       22
#define BOARD_PIN_TP_INT       21

/* Bring up LCD, backlight, touch and LVGL. Returns the LVGL display. */
lv_display_t *board_init(void);

/* 0..100 % */
void board_set_backlight(uint8_t percent);

/* Panel VCOM (JD9365 page 1, register 0x01). Wrong VCOM shows up as image
 * retention ("ghosting"); it can be tuned live from the Settings tab. */
#define BOARD_VCOM_DEFAULT  0x44
#define BOARD_VCOM_MIN      0x10
#define BOARD_VCOM_MAX      0x80
void board_set_vcom(uint8_t vcom);

/* Rotate the whole picture (and touch) by 180 degrees using the panel's own
 * scan-direction bits - no rendering cost. Can be called at any time. */
void board_set_flip(bool flipped);

/* Screen saver: backlight off. The next touch wakes the screen and is swallowed
 * (it does not reach the UI). */
void board_screen_off(void);
void board_screen_wake(void);
bool board_screen_is_off(void);
