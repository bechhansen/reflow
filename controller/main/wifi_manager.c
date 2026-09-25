#include "wifi_manager.h"
#include "cJSON.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs.h"
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include <string.h>

#define AP_SSID          "Reflow-Setup"
#define AP_CHANNEL       6
#define AP_MAX_CONN      4
#define STA_TIMEOUT_MS   15000
#define STA_SCAN_ATTEMPTS 1   /* driver re-cycling does NOT clear the deaf receiver; see below */
#define STA_RETRY_MAX    5
#define NVS_NS           "wifi_cfg"
#define NVS_KEY_SSID     "ssid"
#define NVS_KEY_PASS     "password"
#define NVS_KEY_HOST     "hostname"
#define HOSTNAME_DEFAULT "reflow"

static const char *TAG = "wifi_mgr";

static EventGroupHandle_t s_wifi_events;

/* Survives esp_restart() but is zeroed on power-on reset — exactly the
   lifetime needed to allow one automatic reboot per cold boot. */
RTC_DATA_ATTR static uint32_t s_sta_reboot_attempted;
#define EVT_STA_GOT_IP    BIT0
#define EVT_STA_FAILED    BIT1

static wifi_mgr_mode_t s_mode = WIFI_MGR_DISCONNECTED;
static char            s_ip[20];
static int             s_retry_count;
/* Guards retry logic: set false before esp_wifi_stop() to prevent the
   STA_DISCONNECTED handler from calling esp_wifi_connect() on a stopped driver. */
static volatile bool   s_connecting;

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_connecting) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (!s_connecting) return;
        wifi_event_sta_disconnected_t *dc = (wifi_event_sta_disconnected_t *)data;
        if (s_retry_count < STA_RETRY_MAX) {
            s_retry_count++;
            ESP_LOGW(TAG, "STA disconnected (reason %d), retry %d/%d", dc->reason, s_retry_count, STA_RETRY_MAX);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_events, EVT_STA_FAILED);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        s_mode = WIFI_MGR_STA;
        xEventGroupSetBits(s_wifi_events, EVT_STA_GOT_IP);
        ESP_LOGI(TAG, "STA connected, IP: %s", s_ip);
    }
}

/* Hostname is a DNS label: lowercase letters, digits and hyphens, not leading
   or trailing with a hyphen. Anything else would make the .local name
   unreachable, so reject rather than silently mangle. */
static bool hostname_valid(const char *h)
{
    size_t n = strlen(h);
    if (n == 0 || n > 32) return false;
    if (h[0] == '-' || h[n - 1] == '-') return false;
    for (size_t i = 0; i < n; i++) {
        char c = h[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-';
        if (!ok) return false;
    }
    return true;
}

/* Currently configured SSID (from NVS), for the settings page to display. */
esp_err_t wifi_manager_get_ssid(char *buf, size_t len)
{
    if (!buf || len == 0) return ESP_ERR_INVALID_ARG;
    buf[0] = '\0';

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        size_t sl = len;
        nvs_get_str(nvs, NVS_KEY_SSID, buf, &sl);
        nvs_close(nvs);
    }
    if (buf[0] == '\0' && strlen(CONFIG_REFLOW_WIFI_SSID) > 0)
        strlcpy(buf, CONFIG_REFLOW_WIFI_SSID, len);
    return ESP_OK;
}

esp_err_t wifi_manager_get_hostname(char *buf, size_t len)
{
    if (!buf || len == 0) return ESP_ERR_INVALID_ARG;
    strlcpy(buf, HOSTNAME_DEFAULT, len);

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        char h[40] = {0};
        size_t hl = sizeof(h);
        if (nvs_get_str(nvs, NVS_KEY_HOST, h, &hl) == ESP_OK && hostname_valid(h))
            strlcpy(buf, h, len);
        nvs_close(nvs);
    }
    return ESP_OK;
}

esp_err_t wifi_manager_set_hostname(const char *hostname)
{
    if (!hostname || !hostname_valid(hostname)) return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NS, NVS_READWRITE, &nvs), TAG, "nvs open");
    nvs_set_str(nvs, NVS_KEY_HOST, hostname);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Hostname saved: %s", hostname);
    return ESP_OK;
}

static void start_mdns(void)
{
    char host[40];
    wifi_manager_get_hostname(host, sizeof(host));

    mdns_init();
    mdns_hostname_set(host);
    mdns_instance_name_set("Reflow Controller");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS started — %s.local", host);
}

static esp_err_t start_ap(void)
{
    wifi_config_t cfg = {
        .ap = {
            .ssid            = AP_SSID,
            .ssid_len        = strlen(AP_SSID),
            .channel         = AP_CHANNEL,
            .max_connection  = AP_MAX_CONN,
            .authmode        = WIFI_AUTH_OPEN,
            .beacon_interval = 100,
            .pmf_cfg         = { .required = false, .capable = false },
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set AP mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &cfg), TAG, "set AP config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start AP");
    esp_wifi_set_ps(WIFI_PS_NONE);
    s_mode = WIFI_MGR_AP;
    ESP_LOGI(TAG, "AP started  SSID: %s  (open, no password)", AP_SSID);
    start_mdns();
    return ESP_OK;
}

esp_err_t wifi_manager_init(void)
{
    s_wifi_events = xEventGroupCreate();

    /* esp_netif_init() and esp_event_loop_create_default() called in app_main() */
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set RAM storage");
    /* 40 = 10 dBm (0.25 dBm steps); reduces peak current on marginal USB supplies */
    esp_wifi_set_max_tx_power(40);

    esp_event_handler_instance_t inst_any_wifi, inst_got_ip;
    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            event_handler, NULL, &inst_any_wifi),
        TAG, "register WIFI_EVENT handler");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                            event_handler, NULL, &inst_got_ip),
        TAG, "register IP_EVENT handler");

    nvs_handle_t nvs;
    char ssid[64] = {0};
    char pass[64] = {0};
    bool has_creds = false;

    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        size_t ssid_len = sizeof(ssid), pass_len = sizeof(pass);
        if (nvs_get_str(nvs, NVS_KEY_SSID, ssid, &ssid_len) == ESP_OK && ssid[0] != '\0')
            has_creds = true;
        nvs_get_str(nvs, NVS_KEY_PASS, pass, &pass_len);
        nvs_close(nvs);
    }

    if (!has_creds && strlen(CONFIG_REFLOW_WIFI_SSID) > 0) {
        strlcpy(ssid, CONFIG_REFLOW_WIFI_SSID, sizeof(ssid));
        strlcpy(pass, CONFIG_REFLOW_WIFI_PASSWORD, sizeof(pass));
        has_creds = true;
    }

    if (!has_creds) {
        ESP_LOGI(TAG, "No Wi-Fi credentials — starting AP");
        return start_ap();
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set STA mode");
    /* Do NOT lock protocol here — removing 11ax before start breaks the C6 receive path */
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start STA for scan");
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* Pre-connect scan, with recovery for a deaf receiver.

       On a cold/hard reset the C6 receiver sometimes comes up hearing nothing:
       this scan completes normally but reports 0 APs, and the subsequent
       connect then fails with reason 201 (NO_AP_FOUND) on every retry, even
       with a strong AP on channel 3. The same image connects immediately when
       the boot came from a soft esp_restart(). Full RF calibration
       (ESP_PHY_RF_CAL_FULL) does not prevent it.

       Root cause unidentified; empirically, cycling the driver clears it. So
       treat "0 APs" as a failed bring-up rather than an empty neighbourhood —
       an environment with no 2.4 GHz networks at all would merely retry twice
       and cost ~6 s before the connect attempt proceeds as normal. */
    for (int attempt = 1; attempt <= STA_SCAN_ATTEMPTS; attempt++) {
        wifi_scan_config_t sc = { .scan_type = WIFI_SCAN_TYPE_ACTIVE };
        if (esp_wifi_scan_start(&sc, true) != ESP_OK) {
            ESP_LOGW(TAG, "Scan failed (attempt %d/%d)", attempt, STA_SCAN_ATTEMPTS);
        } else {
            uint16_t n = 0;
            esp_wifi_scan_get_ap_num(&n);
            ESP_LOGI(TAG, "Scan found %u AP(s) (attempt %d/%d):", n, attempt, STA_SCAN_ATTEMPTS);
            if (n > 0) {
                if (n > 20) n = 20;
                wifi_ap_record_t *recs = malloc(n * sizeof(wifi_ap_record_t));
                if (recs) {
                    esp_wifi_scan_get_ap_records(&n, recs);
                    for (int i = 0; i < n; i++)
                        ESP_LOGI(TAG, "  [%3d dBm] ch%-3d %s",
                                 recs[i].rssi, recs[i].primary, (char *)recs[i].ssid);
                    free(recs);
                }
                break;          /* receiver is working — proceed to connect */
            }
        }

        if (attempt == STA_SCAN_ATTEMPTS) break;

        ESP_LOGW(TAG, "Receiver appears deaf — restarting Wi-Fi driver");
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "re-set STA mode");
        ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "re-start STA");
        esp_wifi_set_ps(WIFI_PS_NONE);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    wifi_config_t sta_cfg = {0};
    strlcpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;

    s_retry_count = 0;
    s_connecting  = true;

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg), TAG, "set STA config");
    esp_wifi_connect();

    ESP_LOGI(TAG, "Connecting to SSID: %s ...", ssid);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                                           EVT_STA_GOT_IP | EVT_STA_FAILED,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(STA_TIMEOUT_MS));
    if (bits & EVT_STA_GOT_IP) {
        start_mdns();
        return ESP_OK;
    }

    /* Disable retry before stopping — prevents esp_wifi_connect() on a stopped driver */
    s_connecting = false;

    /* Cold-boot deaf-receiver workaround.
       After a power-on or hard reset the receiver sometimes hears nothing at
       all: the pre-connect scan reports 0 APs and every connect attempt fails
       with reason 201, even with a strong AP a few metres away. Restarting the
       Wi-Fi driver in place does NOT clear it; a chip-level soft restart
       reliably does. Root cause unidentified (ESP32-C6 / IDF 6.1).

       So spend one automatic reboot before falling back to setup mode. The flag
       lives in RTC memory: it survives the esp_restart() below, so the second
       attempt cannot loop, and it is cleared by a power cycle, so the next cold
       boot gets its own retry. */
    if (s_sta_reboot_attempted == 0) {
        s_sta_reboot_attempted = 1;
        ESP_LOGW(TAG, "STA failed on cold boot — soft-restarting once to reset the receiver");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }
    s_sta_reboot_attempted = 0;

    ESP_LOGW(TAG, "STA connection failed after reboot retry — falling back to AP");
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_NULL);
    vTaskDelay(pdMS_TO_TICKS(100));
    return start_ap();
}

wifi_mgr_mode_t wifi_manager_get_mode(void)
{
    return s_mode;
}

esp_err_t wifi_manager_get_ip(char *buf, size_t len)
{
    if (s_mode != WIFI_MGR_STA) return ESP_ERR_INVALID_STATE;
    strlcpy(buf, s_ip, len);
    return ESP_OK;
}

esp_err_t wifi_manager_set_credentials(const char *ssid, const char *password)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NS, NVS_READWRITE, &nvs), TAG, "nvs open");
    nvs_set_str(nvs, NVS_KEY_SSID, ssid);
    nvs_set_str(nvs, NVS_KEY_PASS, password);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Wi-Fi credentials saved for SSID: %s", ssid);
    return ESP_OK;
}

esp_err_t wifi_manager_scan_json(char *buf, size_t buf_len)
{
    /* Scanning requires the STA interface — temporarily enable APSTA if in AP-only
       mode. The mode must stay APSTA until the records have been read: dropping the
       STA interface discards the scan result list, which yields an empty scan. */
    bool apsta = (s_mode == WIFI_MGR_AP);
    if (apsta) esp_wifi_set_mode(WIFI_MODE_APSTA);

    wifi_scan_config_t scan_cfg = { .show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE };
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);

    if (err != ESP_OK) {
        if (apsta) esp_wifi_set_mode(WIFI_MODE_AP);
        ESP_LOGW(TAG, "scan start failed: %s", esp_err_to_name(err));
        strlcpy(buf, "[]", buf_len);
        return err;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    ESP_LOGI(TAG, "scan found %u AP(s)", ap_count);
    if (ap_count > 20) ap_count = 20;

    wifi_ap_record_t *records = NULL;
    if (ap_count > 0) {
        records = malloc(ap_count * sizeof(wifi_ap_record_t));
        if (!records) {
            if (apsta) esp_wifi_set_mode(WIFI_MODE_AP);
            strlcpy(buf, "[]", buf_len);
            return ESP_ERR_NO_MEM;
        }
        esp_wifi_scan_get_ap_records(&ap_count, records);
    }

    if (apsta) esp_wifi_set_mode(WIFI_MODE_AP);

    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < ap_count; i++) {
        cJSON *obj = cJSON_CreateObject();
        cJSON_AddStringToObject(obj, "ssid", (char *)records[i].ssid);
        cJSON_AddNumberToObject(obj, "rssi", records[i].rssi);
        cJSON_AddNumberToObject(obj, "ch",   records[i].primary);
        ESP_LOGI(TAG, "  [%3d dBm] ch%-3d %s",
                 records[i].rssi, records[i].primary, (char *)records[i].ssid);
        cJSON_AddItemToArray(arr, obj);
    }
    free(records);

    char *str = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if (!str) { strlcpy(buf, "[]", buf_len); return ESP_ERR_NO_MEM; }
    strlcpy(buf, str, buf_len);
    free(str);
    return ESP_OK;
}
