#include "wifi.h"

#include <string.h>
#include <time.h>
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "settings.h"

static const char *TAG = "wifi";

#define BIT_CONNECTED  BIT0
#define BIT_SCAN_DONE  BIT1

static EventGroupHandle_t s_ev;
static esp_netif_t *s_netif;
static volatile bool s_started;
static volatile bool s_scanning;
static volatile int s_retry;
static const char *s_state = "Starting";
static wifi_ap_info_t s_scan[WIFI_SCAN_MAX];
static int s_scan_count = -1;
static bool s_sntp_started;
static esp_timer_handle_t s_reconnect_timer;

static void reconnect_cb(void *arg)
{
    if (settings_get()->wifi_ssid[0] && !s_scanning) esp_wifi_connect();
}

static void start_sntp(void)
{
    if (s_sntp_started) return;
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&cfg);
    s_sntp_started = true;
}

bool time_is_synced(void)
{
    return time(NULL) > 1700000000;  /* after Nov 2023 => synced */
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        s_started = true;
        if (settings_get()->wifi_ssid[0]) {
            s_state = "Connecting";
            esp_wifi_connect();
        } else {
            s_state = "No SSID set";
        }
        break;
    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *d = data;
        xEventGroupClearBits(s_ev, BIT_CONNECTED);
        ESP_LOGW(TAG, "disconnected (reason %d)", d ? d->reason : -1);
        s_state = "Reconnecting";
        if (settings_get()->wifi_ssid[0] && !s_scanning) {
            /* back-off 1 s .. 10 s without blocking the event loop */
            int delay_ms = s_retry < 10 ? (s_retry + 1) * 1000 : 10000;
            s_retry++;
            esp_timer_stop(s_reconnect_timer);
            esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_ms * 1000);
        }
        break;
    }
    case WIFI_EVENT_SCAN_DONE: {
        uint16_t n = WIFI_SCAN_MAX;
        wifi_ap_record_t *recs = calloc(WIFI_SCAN_MAX, sizeof(wifi_ap_record_t));
        int count = 0;
        if (recs && esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
            for (int i = 0; i < n; i++) {
                if (!recs[i].ssid[0]) continue;
                bool dup = false;
                for (int j = 0; j < count; j++) {
                    if (strcmp(s_scan[j].ssid, (char *)recs[i].ssid) == 0) { dup = true; break; }
                }
                if (dup) continue;
                strlcpy(s_scan[count].ssid, (char *)recs[i].ssid, sizeof(s_scan[count].ssid));
                s_scan[count].rssi = recs[i].rssi;
                s_scan[count].secure = recs[i].authmode != WIFI_AUTH_OPEN;
                count++;
            }
        }
        free(recs);
        s_scan_count = count;
        s_scanning = false;
        xEventGroupSetBits(s_ev, BIT_SCAN_DONE);
        if (!wifi_is_connected() && settings_get()->wifi_ssid[0]) esp_wifi_connect();
        break;
    }
    default:
        break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "got IP " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry = 0;
        s_state = "Connected";
        xEventGroupSetBits(s_ev, BIT_CONNECTED);
        start_sntp();
    }
}

static void apply_config(void)
{
    const app_settings_t *c = settings_get();
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, c->wifi_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, c->wifi_pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = c->wifi_pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    wc.sta.pmf_cfg.capable = true;
    wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wc.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    esp_wifi_set_config(WIFI_IF_STA, &wc);
}

void wifi_init(void)
{
    s_ev = xEventGroupCreate();
    const esp_timer_create_args_t targs = {.callback = reconnect_cb, .name = "wifi_rc"};
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_reconnect_timer));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    /* esp_wifi_remote routes these calls to the ESP32-C6 over SDIO (esp_hosted) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s (is the C6 running esp-hosted firmware?)", esp_err_to_name(err));
        s_state = "C6 not responding";
        return;
    }
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, NULL));
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    apply_config();
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_netif_set_hostname(s_netif, CONFIG_LWIP_LOCAL_HOSTNAME);   /* router adds its domain */
}

void wifi_apply_settings(void)
{
    if (!s_started) return;
    s_retry = 0;
    xEventGroupClearBits(s_ev, BIT_CONNECTED);
    esp_wifi_disconnect();
    apply_config();
    if (settings_get()->wifi_ssid[0]) {
        s_state = "Connecting";
        esp_wifi_connect();
    } else {
        s_state = "No SSID set";
    }
}

bool wifi_is_connected(void)
{
    return s_ev && (xEventGroupGetBits(s_ev) & BIT_CONNECTED);
}

bool wifi_wait_connected(TickType_t timeout)
{
    if (!s_ev) return false;
    return xEventGroupWaitBits(s_ev, BIT_CONNECTED, pdFALSE, pdTRUE, timeout) & BIT_CONNECTED;
}

int wifi_rssi(void)
{
    if (!wifi_is_connected()) return 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
    return 0;
}

void wifi_ip_str(char *buf, int len)
{
    esp_netif_ip_info_t ip;
    if (s_netif && esp_netif_get_ip_info(s_netif, &ip) == ESP_OK && ip.ip.addr) {
        snprintf(buf, len, IPSTR, IP2STR(&ip.ip));
    } else {
        strlcpy(buf, "-", len);
    }
}

const char *wifi_state_str(void)
{
    return s_state;
}

void wifi_scan_start(void)
{
    if (!s_started || s_scanning) return;
    s_scanning = true;
    s_scan_count = -1;
    xEventGroupClearBits(s_ev, BIT_SCAN_DONE);
    /* scanning while a connect attempt is running fails, so pause it */
    if (!wifi_is_connected()) esp_wifi_disconnect();
    wifi_scan_config_t sc = {.show_hidden = false};
    if (esp_wifi_scan_start(&sc, false) != ESP_OK) {
        s_scanning = false;
        s_scan_count = 0;
    }
}

int wifi_scan_results(wifi_ap_info_t *out, int max)
{
    if (s_scan_count < 0) return -1;
    int n = s_scan_count < max ? s_scan_count : max;
    memcpy(out, s_scan, n * sizeof(wifi_ap_info_t));
    return n;
}
