#pragma once

/*
 * Demo mode (CONFIG_ADSB_DEMO): simulated traffic around London Heathrow for
 * screenshots and shop-window use, plus GET /screenshot.bmp on the update web
 * server. The simulated aircraft, registrations and routes are illustrative.
 */

/* Override settings in RAM (location, units, timezone...). Call after settings_init(). */
void demo_prepare_settings(void);
/* Start the simulator and register the screenshot handler (after ota_start_server()). */
void demo_start(void);
