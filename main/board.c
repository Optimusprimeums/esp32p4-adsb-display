#include "board.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_lcd_jd9365.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_gsl3680.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "panel_jc8012_v2.h"
#include "settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "board";

/* Kconfig bools are undefined when "n" */
#ifdef CONFIG_ADSB_LCD_MIRROR_X
#define LCD_MIRROR_X true
#else
#define LCD_MIRROR_X false
#endif
#ifdef CONFIG_ADSB_LCD_MIRROR_Y
#define LCD_MIRROR_Y true
#else
#define LCD_MIRROR_Y false
#endif
#ifdef CONFIG_ADSB_TOUCH_MIRROR_X
#define TOUCH_MIRROR_X 1
#else
#define TOUCH_MIRROR_X 0
#endif
#ifdef CONFIG_ADSB_TOUCH_MIRROR_Y
#define TOUCH_MIRROR_Y 1
#else
#define TOUCH_MIRROR_Y 0
#endif

/* VDD_MIPI_DPHY is fed from the P4's internal LDO channel 3 at 2.5 V */
#define DSI_PHY_LDO_CHAN       3
#define DSI_PHY_LDO_MV         2500
#define DSI_LANES              2
#define DSI_LANE_MBPS          840
#define DPI_CLOCK_MHZ          70

#define BL_LEDC_TIMER          LEDC_TIMER_0
#define BL_LEDC_CHANNEL        LEDC_CHANNEL_0
#define BL_LEDC_RES            LEDC_TIMER_10_BIT

static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_touch_handle_t s_touch;
static SemaphoreHandle_t s_vsync_sem;

/* ------------------------------------------------------------------------- */
/* Backlight                                                                 */
/* ------------------------------------------------------------------------- */

static void backlight_init(void)
{
    const ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BL_LEDC_RES,
        .timer_num = BL_LEDC_TIMER,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
    const ledc_channel_config_t ccfg = {
        .gpio_num = BOARD_PIN_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BL_LEDC_CHANNEL,
        .timer_sel = BL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
}

void board_set_flip(bool flipped)
{
    if (s_panel) {
        esp_lcd_panel_mirror(s_panel, LCD_MIRROR_X != flipped, LCD_MIRROR_Y != flipped);
    }
    if (s_touch) {
        esp_lcd_touch_set_mirror_x(s_touch, (bool)TOUCH_MIRROR_X != flipped);
        esp_lcd_touch_set_mirror_y(s_touch, (bool)TOUCH_MIRROR_Y != flipped);
    }
    ESP_LOGI(TAG, "screen orientation: %s", flipped ? "flipped 180" : "normal");
}

void board_set_vcom(uint8_t vcom)
{
    if (!s_io) return;
    if (vcom < BOARD_VCOM_MIN) vcom = BOARD_VCOM_MIN;
    if (vcom > BOARD_VCOM_MAX) vcom = BOARD_VCOM_MAX;
    /* page 1 holds the power/VCOM registers; always return to the user page */
    esp_lcd_panel_io_tx_param(s_io, 0xE0, (uint8_t[]){0x01}, 1);
    esp_lcd_panel_io_tx_param(s_io, 0x01, (uint8_t[]){vcom}, 1);
    esp_lcd_panel_io_tx_param(s_io, 0xE0, (uint8_t[]){0x00}, 1);
    ESP_LOGI(TAG, "panel VCOM = 0x%02X", vcom);
}

static uint8_t s_brightness = 80;
static volatile bool s_screen_off;
static bool s_swallow_touch;

static void backlight_write(uint8_t percent)
{
    uint32_t duty = ((1u << BL_LEDC_RES) - 1) * percent / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
}

void board_screen_off(void)
{
    if (s_screen_off) return;
    s_screen_off = true;
    backlight_write(0);
    ESP_LOGI(TAG, "screen saver: display off");
}

void board_screen_wake(void)
{
    if (!s_screen_off) return;
    s_screen_off = false;
    backlight_write(s_brightness);
    lv_display_trigger_activity(NULL);
    ESP_LOGI(TAG, "screen saver: display on");
}

bool board_screen_is_off(void)
{
    return s_screen_off;
}

void board_set_backlight(uint8_t percent)
{
    if (percent > 100) percent = 100;
    s_brightness = percent;
    if (s_screen_off) return;   /* applied on wake */
    uint32_t duty = ((1u << BL_LEDC_RES) - 1) * percent / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
}

/* ------------------------------------------------------------------------- */
/* LCD                                                                       */
/* ------------------------------------------------------------------------- */

static bool IRAM_ATTR on_vsync(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync_sem, &woken);
    return woken == pdTRUE;
}

static void lcd_init(void)
{
    esp_ldo_channel_handle_t ldo = NULL;
    const esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = DSI_PHY_LDO_CHAN,
        .voltage_mv = DSI_PHY_LDO_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo));

    esp_lcd_dsi_bus_handle_t dsi_bus;
    const esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id = 0,
        .num_data_lanes = DSI_LANES,
        .phy_clk_src = 0,      /* default */
        .lane_bit_rate_mbps = DSI_LANE_MBPS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));

    esp_lcd_panel_io_handle_t io;
    const esp_lcd_dbi_io_config_t dbi_cfg = JD9365_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &io));

    /* "V2" panel timing (fitted to boards with ESP32-P4 v3.x), from jtenniswood/espcontrol */
    const esp_lcd_dpi_panel_config_t dpi_cfg = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = DPI_CLOCK_MHZ,
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .out_color_format = LCD_COLOR_FMT_RGB565,
        .num_fbs = 2,          /* LVGL renders directly into these, flipped on vsync */
        .video_timing = {
            .h_size = BOARD_LCD_H_RES,
            .v_size = BOARD_LCD_V_RES,
            .hsync_pulse_width = 20,
            .hsync_back_porch = 20,
            .hsync_front_porch = 40,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 10,
            .vsync_front_porch = 30,
        },
    };
    jd9365_vendor_config_t vendor_cfg = {
        .init_cmds = jc8012_v2_init_cmds,
        .init_cmds_size = sizeof(jc8012_v2_init_cmds) / sizeof(jc8012_v2_init_cmds[0]),
        .mipi_config = {
            .dsi_bus = dsi_bus,
            .dpi_config = &dpi_cfg,
            .lane_num = DSI_LANES,
        },
    };
    const esp_lcd_panel_dev_config_t dev_cfg = {
        .reset_gpio_num = BOARD_PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9365(io, &dev_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, LCD_MIRROR_X, LCD_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    s_io = io;
    board_set_vcom(settings_get()->vcom);

    s_vsync_sem = xSemaphoreCreateBinary();
    const esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_vsync = on_vsync,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL));
    ESP_LOGI(TAG, "LCD up: %dx%d", BOARD_LCD_H_RES, BOARD_LCD_V_RES);
}

/* ------------------------------------------------------------------------- */
/* Touch                                                                     */
/* ------------------------------------------------------------------------- */

static void touch_init(void)
{
    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = 0,
        .sda_io_num = BOARD_PIN_TP_SDA,
        .scl_io_num = BOARD_PIN_TP_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    esp_lcd_panel_io_handle_t tp_io;
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_GSL3680_CONFIG();
    io_cfg.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus, &io_cfg, &tp_io));

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BOARD_LCD_H_RES,
        .y_max = BOARD_LCD_V_RES,
        .rst_gpio_num = BOARD_PIN_TP_RST,
        .int_gpio_num = BOARD_PIN_TP_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = TOUCH_MIRROR_X,
            .mirror_y = TOUCH_MIRROR_Y,
        },
    };
    esp_err_t err = esp_lcd_touch_new_i2c_gsl3680(tp_io, &tp_cfg, &s_touch);
    if (err != ESP_OK) {
        /* keep running without touch rather than boot-looping */
        ESP_LOGE(TAG, "touch init failed: %s", esp_err_to_name(err));
        s_touch = NULL;
    }
}

/* ------------------------------------------------------------------------- */
/* LVGL glue                                                                 */
/* ------------------------------------------------------------------------- */

static uint32_t lv_tick_cb(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (lv_display_flush_is_last(disp)) {
        /* px_map is one of the DPI frame buffers: this only does cache write-back
         * and tells the DPI engine to scan out from it. Wait for the next vsync so
         * LVGL doesn't start drawing into the buffer that's still on screen. */
        xSemaphoreTake(s_vsync_sem, 0);
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, BOARD_LCD_H_RES, BOARD_LCD_V_RES, px_map);
        xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(50));
    }
    lv_display_flush_ready(disp);
}

static void lv_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t x[1], y[1], strength[1];
    uint8_t cnt = 0;

    if (s_touch == NULL) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }
    esp_lcd_touch_read_data(s_touch);
    bool pressed = esp_lcd_touch_get_coordinates(s_touch, x, y, strength, &cnt, 1) && cnt > 0;

    /* screen saver: a touch while dark only wakes the screen; ignore it until lifted */
    if (s_screen_off && pressed) {
        board_screen_wake();
        s_swallow_touch = true;
    }
    if (s_swallow_touch) {
        if (!pressed) s_swallow_touch = false;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }
    if (pressed && cnt > 0) {
        data->point.x = x[0] < BOARD_LCD_H_RES ? x[0] : BOARD_LCD_H_RES - 1;
        data->point.y = y[0] < BOARD_LCD_V_RES ? y[0] : BOARD_LCD_V_RES - 1;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

lv_display_t *board_init(void)
{
    backlight_init();
    board_set_backlight(0);
    lcd_init();
    touch_init();

    if (settings_get()->flip) board_set_flip(true);

    lv_init();
    lv_tick_set_cb(lv_tick_cb);

    lv_display_t *disp = lv_display_create(BOARD_LCD_H_RES, BOARD_LCD_V_RES);
    void *fb0 = NULL, *fb1 = NULL;
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &fb0, &fb1));
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(disp, fb0, fb1, BOARD_LCD_H_RES * BOARD_LCD_V_RES * 2,
                           LV_DISPLAY_RENDER_MODE_DIRECT);
    lv_display_set_flush_cb(disp, lv_flush_cb);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lv_touch_read_cb);
    lv_indev_set_display(indev, disp);

    return disp;
}
