#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Firmware updates.
 *
 * P4 application (A/B slots, ota_0 / ota_1):
 *   - Upload a .bin over Wi-Fi via http://<device-ip>/ (or curl --data-binary)
 *     while the upload window is open (armed from the Settings tab, 10 min).
 *   - The image is written to the inactive slot and fully verified (header,
 *     project name, chip, SHA-256) before it is made bootable. A failed or
 *     interrupted upload leaves the running firmware untouched.
 *   - A new image boots "pending verify". It is confirmed only after the
 *     decoder self-test passes, the UI is up and Wi-Fi connects (within
 *     OTA_VERIFY_TIMEOUT_S). A crash/watchdog reset or failed check before
 *     that makes the bootloader return to the previous image.
 *   - The previous image can also be re-selected manually.
 *
 * ESP32-C6 Wi-Fi co-processor:
 *   - The matching ESP-Hosted co-processor firmware is embedded in this app
 *     and can be pushed to the C6 over SDIO from the Settings tab.
 */

#define OTA_VERIFY_TIMEOUT_S   300
#define OTA_ARM_SECONDS        600

typedef enum {
    OTA_IDLE = 0,
    OTA_RECEIVING,
    OTA_DONE_REBOOTING,
    OTA_FAILED,
} ota_xfer_state_t;

typedef struct {
    /* running image */
    char version[32];
    char build_date[24];
    char partition[16];
    bool pending_verify;            /* new image not yet confirmed */
    /* previous image (other slot) */
    bool can_revert;
    char prev_version[32];
    /* upload */
    uint32_t armed_left_s;          /* 0 = uploads refused */
    ota_xfer_state_t state;
    uint32_t received, total;
    char message[96];
    /* ESP32-C6 */
    bool c6_version_known;
    char c6_version[16];            /* running on the C6 */
    char c6_bundled[16];            /* embedded in this firmware */
    bool c6_update_available;
    bool c6_busy;
    uint32_t c6_sent, c6_total;
    char c6_message[96];
} ota_status_t;

/* Call early in app_main (after NVS). Starts boot verification if needed. */
void ota_init(void);
/* Start the web server (after networking is initialised). */
void ota_start_server(void);

/* Inputs to the boot verification */
void ota_report_selftest(bool passed);
void ota_report_ui_ready(void);

/* Open the upload window for OTA_ARM_SECONDS */
void ota_arm(void);
/* Switch back to the image in the other slot and reboot */
bool ota_revert(void);
/* Push the bundled firmware to the ESP32-C6 (runs in background, then reboots) */
bool ota_c6_update_start(void);

void ota_get_status(ota_status_t *out);
