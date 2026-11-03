#include "ota.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_hosted.h"
#include "esp_hosted_ota.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_image_format.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "wifi.h"

static const char *TAG = "ota";

/* ESP-Hosted co-processor firmware bundled into this app (built from
 * c6_coprocessor/, pinned to the same ESP-Hosted release as the host side). */
#define C6_BUNDLED_MAJOR 3
#define C6_BUNDLED_MINOR 0
#define C6_BUNDLED_PATCH 9
extern const uint8_t c6_fw_start[] asm("_binary_c6_hosted_coprocessor_bin_start");
extern const uint8_t c6_fw_end[]   asm("_binary_c6_hosted_coprocessor_bin_end");

#define RX_CHUNK 4096

static ota_status_t s_st;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_armed_until_us;
static volatile bool s_selftest_ok, s_selftest_done, s_ui_ready;
static httpd_handle_t s_httpd;

static void set_msg(char *dst, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void set_msg(char *dst, const char *fmt, ...)
{
    char tmp[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    portENTER_CRITICAL(&s_lock);
    strlcpy(dst, tmp, 96);
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "%s", tmp);
}

/* ------------------------------------------------------------------------- */
/* Boot verification / rollback                                              */
/* ------------------------------------------------------------------------- */

void ota_report_selftest(bool passed)
{
    s_selftest_ok = passed;
    s_selftest_done = true;
}

void ota_report_ui_ready(void)
{
    s_ui_ready = true;
}

static void verify_task(void *arg)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)OTA_VERIFY_TIMEOUT_S * 1000000;
    ESP_LOGW(TAG, "new firmware is pending verification (self-test, UI, Wi-Fi within %d s)",
             OTA_VERIFY_TIMEOUT_S);
    for (;;) {
        if (s_selftest_done && !s_selftest_ok) {
            ESP_LOGE(TAG, "self-test failed: rolling back to previous firmware");
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
        if (s_selftest_ok && s_ui_ready && wifi_is_connected()) {
            esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "firmware verified and confirmed (%s)", esp_err_to_name(err));
            portENTER_CRITICAL(&s_lock);
            s_st.pending_verify = false;
            portEXIT_CRITICAL(&s_lock);
            break;
        }
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "firmware not verified within %d s: rolling back", OTA_VERIFY_TIMEOUT_S);
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

static void refresh_partition_info(void)
{
    const esp_app_desc_t *d = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    esp_app_desc_t prev;
    esp_ota_img_states_t ost = ESP_OTA_IMG_UNDEFINED;
    /* the other slot only counts if it holds a complete, verifiable image
     * (an interrupted upload leaves a valid header but a truncated body) */
    esp_partition_pos_t pos = {.offset = other ? other->address : 0, .size = other ? other->size : 0};
    esp_image_metadata_t meta;
    bool prev_ok = other && esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &pos, &meta) == ESP_OK &&
                   esp_ota_get_partition_description(other, &prev) == ESP_OK &&
                   (esp_ota_get_state_partition(other, &ost) != ESP_OK ||
                    (ost != ESP_OTA_IMG_INVALID && ost != ESP_OTA_IMG_ABORTED));

    portENTER_CRITICAL(&s_lock);
    strlcpy(s_st.version, d->version, sizeof(s_st.version));
    snprintf(s_st.build_date, sizeof(s_st.build_date), "%s %.5s", d->date, d->time);
    strlcpy(s_st.partition, run ? run->label : "?", sizeof(s_st.partition));
    s_st.can_revert = prev_ok;
    if (prev_ok) strlcpy(s_st.prev_version, prev.version, sizeof(s_st.prev_version));
    else s_st.prev_version[0] = 0;
    portEXIT_CRITICAL(&s_lock);
}

void ota_init(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    snprintf(s_st.c6_bundled, sizeof(s_st.c6_bundled), "%d.%d.%d",
             C6_BUNDLED_MAJOR, C6_BUNDLED_MINOR, C6_BUNDLED_PATCH);
    refresh_partition_info();
    ESP_LOGI(TAG, "running %s from %s", s_st.version, run ? run->label : "?");

    if (run && esp_ota_get_state_partition(run, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
        s_st.pending_verify = true;
        xTaskCreate(verify_task, "ota_verify", 3072, NULL, 5, NULL);
    }
}

bool ota_revert(void)
{
    refresh_partition_info();
    if (!s_st.can_revert) return false;
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    if (esp_ota_set_boot_partition(other) != ESP_OK) return false;
    ESP_LOGW(TAG, "reverting to %s in %s", s_st.prev_version, other->label);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return true;
}

/* ------------------------------------------------------------------------- */
/* Web upload                                                                */
/* ------------------------------------------------------------------------- */

void ota_arm(void)
{
    s_armed_until_us = esp_timer_get_time() + (int64_t)OTA_ARM_SECONDS * 1000000;
    set_msg(s_st.message, "Upload window open for %d minutes", OTA_ARM_SECONDS / 60);
}

static bool armed(void)
{
    return esp_timer_get_time() < s_armed_until_us;
}

static const char UPLOAD_PAGE[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ADS-B display update</title><style>"
    "body{font-family:system-ui,sans-serif;background:#0b1220;color:#dce6f5;max-width:560px;margin:40px auto;padding:0 16px}"
    "h1{font-size:22px}.box{background:#131c2e;border-radius:10px;padding:18px;margin:14px 0}"
    "button{background:#22c55e;color:#000;border:0;border-radius:8px;padding:10px 18px;font-size:16px}"
    "progress{width:100%;height:18px}#msg{margin-top:10px;white-space:pre-wrap}"
    "</style></head><body><h1>ADS-B display firmware update</h1>"
    "<div class='box' id='info'>Loading...</div>"
    "<div class='box'><input type='file' id='f' accept='.bin'><br><br>"
    "<button onclick='up()'>Upload and install</button><br><br>"
    "<progress id='p' value='0' max='100'></progress><div id='msg'></div></div>"
    "<script>"
    "fetch('/status').then(r=>r.json()).then(s=>{document.getElementById('info').innerText="
    "'Running: '+s.version+' ('+s.partition+')\\nPrevious: '+(s.prev||'none')+"
    "'\\nUpload window: '+(s.armed>0?('open, '+s.armed+' s left'):'CLOSED - tap \"Allow OTA update\" in Settings on the display')});"
    "function up(){var f=document.getElementById('f').files[0];if(!f){alert('Choose adsb_display.bin');return;}"
    "var x=new XMLHttpRequest();x.open('POST','/update');"
    "x.upload.onprogress=e=>{if(e.lengthComputable)document.getElementById('p').value=e.loaded*100/e.total;};"
    "x.onload=()=>{document.getElementById('msg').innerText=x.responseText;};"
    "x.onerror=()=>{document.getElementById('msg').innerText='Upload failed (connection lost)';};"
    "x.send(f);}"
    "</script></body></html>";

static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, UPLOAD_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get(httpd_req_t *req)
{
    ota_status_t st;
    ota_get_status(&st);
    char buf[256];
    snprintf(buf, sizeof(buf), "{\"version\":\"%s\",\"partition\":\"%s\",\"prev\":\"%s\",\"armed\":%lu,"
             "\"pending_verify\":%s}", st.version, st.partition, st.prev_version,
             (unsigned long)st.armed_left_s, st.pending_verify ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t fail(httpd_req_t *req, const char *status, const char *msg)
{
    set_msg(s_st.message, "Update failed: %s", msg);
    s_st.state = OTA_FAILED;
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    char buf[128];
    snprintf(buf, sizeof(buf), "Update failed: %s\nThe current firmware is unchanged.", msg);
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* Validate the start of an uploaded image before anything is written */
static const char *check_image_head(const uint8_t *p, size_t n)
{
    const size_t need = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
    if (n < need) return "file too small";
    const esp_image_header_t *h = (const esp_image_header_t *)p;
    if (h->magic != ESP_IMAGE_HEADER_MAGIC) return "not an ESP firmware image (.bin)";
    if (h->chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID) return "image is for a different chip";
    const esp_app_desc_t *d = (const esp_app_desc_t *)(p + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
    if (d->magic_word != ESP_APP_DESC_MAGIC_WORD) return "image has no app description";
    if (strncmp(d->project_name, esp_app_get_description()->project_name, sizeof(d->project_name)) != 0) {
        return "image is a different project (expected adsb_display.bin)";
    }
    return NULL;
}

/* During an upload the network tasks can keep a core busy for many seconds,
 * starving its idle task. That is load, not a hang, so relax the task watchdog
 * (no idle-task check, long timeout) for the transfer and restore it after. */
static void wdt_relax(void)
{
    const esp_task_wdt_config_t cfg = {.timeout_ms = 60000, .idle_core_mask = 0, .trigger_panic = true};
    esp_task_wdt_reconfigure(&cfg);
}

static void wdt_restore(void)
{
    uint32_t mask = 0;
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
    mask |= 1u << 0;
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
    mask |= 1u << 1;
#endif
    const esp_task_wdt_config_t cfg = {
        .timeout_ms = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = mask,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&cfg);
}

static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

static esp_err_t update_post(httpd_req_t *req)
{
    if (!armed()) {
        return fail(req, "403 Forbidden", "upload window closed - tap \"Allow OTA update\" in Settings on the display first");
    }
    if (s_st.state == OTA_RECEIVING || s_st.c6_busy) {
        return fail(req, "409 Conflict", "another update is in progress");
    }
    if (s_st.pending_verify) {
        return fail(req, "409 Conflict", "running firmware is not verified yet - wait until Wi-Fi is up");
    }
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) return fail(req, "500 Internal Server Error", "no OTA partition");
    if (req->content_len == 0 || req->content_len > target->size) {
        return fail(req, "400 Bad Request", "image size invalid or larger than the OTA slot");
    }

    uint8_t *buf = malloc(RX_CHUNK);
    if (!buf) return fail(req, "500 Internal Server Error", "out of memory");

    s_st.state = OTA_RECEIVING;
    wdt_relax();
    s_st.received = 0;
    s_st.total = req->content_len;
    set_msg(s_st.message, "Receiving firmware into %s...", target->label);

    esp_ota_handle_t h = 0;
    bool begun = false;
    size_t left = req->content_len;
    const char *err = NULL;
    int timeouts = 0;

    while (left > 0) {
        int r = httpd_req_recv(req, (char *)buf, left < RX_CHUNK ? left : RX_CHUNK);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts > 10) { err = "upload timed out"; break; }
            continue;
        }
        if (r <= 0) { err = "connection lost during upload"; break; }
        timeouts = 0;

        if (!begun) {
            /* the first chunk carries the image header and app description */
            err = check_image_head(buf, r);
            if (err) break;
            if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK) {
                err = "could not start writing the OTA slot";
                break;
            }
            begun = true;
        }
        if (esp_ota_write(h, buf, r) != ESP_OK) { err = "flash write failed"; break; }
        /* Over a fast LAN recv() never blocks, so without this the httpd task
         * would hog its core and the idle task's watchdog would reset the board. */
        vTaskDelay(1);
        left -= r;
        s_st.received += r;
    }
    free(buf);
    wdt_restore();

    if (err) {
        if (begun) esp_ota_abort(h);
        return fail(req, "400 Bad Request", err);
    }

    /* esp_ota_end verifies the complete image, including its SHA-256 */
    esp_err_t e = esp_ota_end(h);
    if (e != ESP_OK) {
        return fail(req, "400 Bad Request", e == ESP_ERR_OTA_VALIDATE_FAILED ?
                    "image verification failed (corrupt or incomplete file)" : "could not finish writing");
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        return fail(req, "500 Internal Server Error", "could not select the new firmware");
    }

    s_st.state = OTA_DONE_REBOOTING;
    s_armed_until_us = 0;
    set_msg(s_st.message, "Update written to %s - rebooting", target->label);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Update installed. The display is rebooting into the new firmware.\n"
                            "It confirms itself once Wi-Fi is up; if it fails it returns to the previous version.");
    xTaskCreate(reboot_task, "ota_reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

void ota_start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.recv_wait_timeout = 10;
    cfg.lru_purge_enable = true;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "web server failed to start");
        return;
    }
    const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = page_get},
        {.uri = "/status", .method = HTTP_GET, .handler = status_get},
        {.uri = "/update", .method = HTTP_POST, .handler = update_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) httpd_register_uri_handler(s_httpd, &uris[i]);
    ESP_LOGI(TAG, "update page on port 80");
}

/* ------------------------------------------------------------------------- */
/* ESP32-C6 co-processor update                                              */
/* ------------------------------------------------------------------------- */

static bool c6_query_version(void)
{
    esp_hosted_coprocessor_fwver_t v = {0};
    if (esp_hosted_get_coprocessor_fwversion(&v) != ESP_OK) return false;
    portENTER_CRITICAL(&s_lock);
    snprintf(s_st.c6_version, sizeof(s_st.c6_version), "%lu.%lu.%lu",
             (unsigned long)v.major1, (unsigned long)v.minor1, (unsigned long)v.patch1);
    s_st.c6_version_known = true;
    uint32_t run = (v.major1 << 16) | (v.minor1 << 8) | v.patch1;
    uint32_t bundled = (C6_BUNDLED_MAJOR << 16) | (C6_BUNDLED_MINOR << 8) | C6_BUNDLED_PATCH;
    s_st.c6_update_available = run < bundled;
    portEXIT_CRITICAL(&s_lock);
    return true;
}

static void c6_update_task(void *arg)
{
    const uint8_t *img = c6_fw_start;
    size_t len = c6_fw_end - c6_fw_start;
    const size_t chunk = 1500;   /* RPC payload size used by ESP-Hosted's own example */
    bool activate = false;

    s_st.c6_sent = 0;
    s_st.c6_total = len;
    set_msg(s_st.c6_message, "Updating C6: sending %u KB...", (unsigned)(len / 1024));

    /* activate RPC exists on co-processor firmware 2.6.0 and newer */
    esp_hosted_coprocessor_fwver_t v = {0};
    if (esp_hosted_get_coprocessor_fwversion(&v) == ESP_OK) {
        activate = v.major1 > 2 || (v.major1 == 2 && v.minor1 >= 6);
    }

    if (esp_hosted_slave_ota_begin() != ESP_OK) {
        set_msg(s_st.c6_message, "C6 update failed to start - C6 firmware unchanged");
        goto out;
    }
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = len - off < chunk ? len - off : chunk;
        if (esp_hosted_slave_ota_write(img + off, n) != ESP_OK) {
            esp_hosted_slave_ota_end();
            set_msg(s_st.c6_message, "C6 update failed at %u%% - C6 firmware unchanged",
                    (unsigned)(off * 100 / len));
            goto out;
        }
        s_st.c6_sent = off + n;
    }
    /* end: the C6 verifies the image before switching to it */
    if (esp_hosted_slave_ota_end() != ESP_OK) {
        set_msg(s_st.c6_message, "C6 rejected the image - C6 firmware unchanged");
        goto out;
    }
    if (activate && esp_hosted_slave_ota_activate() != ESP_OK) {
        set_msg(s_st.c6_message, "C6 could not activate the image - C6 firmware unchanged");
        goto out;
    }
    set_msg(s_st.c6_message, "C6 updated - restarting to reconnect");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();

out:
    s_st.c6_busy = false;
    vTaskDelete(NULL);
}

bool ota_c6_update_start(void)
{
    if (s_st.c6_busy || s_st.state == OTA_RECEIVING || s_st.pending_verify) return false;
    s_st.c6_busy = true;
    if (xTaskCreate(c6_update_task, "c6_ota", 6144, NULL, 4, NULL) != pdPASS) {
        s_st.c6_busy = false;
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------- */

void ota_get_status(ota_status_t *out)
{
    static int64_t last_c6_query;
    int64_t now = esp_timer_get_time();
    /* the C6 version only changes after an update+reboot: ask once Wi-Fi is up */
    if (!s_st.c6_version_known && !s_st.c6_busy && wifi_is_connected() && now - last_c6_query > 10000000) {
        last_c6_query = now;
        c6_query_version();
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_st;
    portEXIT_CRITICAL(&s_lock);
    int64_t left = (s_armed_until_us - now) / 1000000;
    out->armed_left_s = left > 0 ? (uint32_t)left : 0;
}
