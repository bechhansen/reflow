#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_coexist.h"

#include "temperature.h"
#include "zigbee_plug.h"
#include "reflow_profile.h"
#include "reflow_ctrl.h"
#include "wifi_manager.h"
#include "web_server.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "main";

/* ── UART console task ───────────────────────────────────────────────────── */

static void console_task(void *arg)
{
    static const char *HELP = "Commands: start <profile>, stop, status, wifi, wifi-scan, wifi-clear\r\n";
    char line[128];
    printf("\r\nReflow console ready. %s", HELP);
    while (fgets(line, sizeof(line), stdin)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) line[--len] = '\0';
        if (len == 0) continue;

        if (strncmp(line, "start ", 6) == 0) {
            reflow_profile_t prof;
            if (profile_get_by_name(line + 6, &prof) == ESP_OK) {
                esp_err_t e = reflow_ctrl_start(&prof);
                if (e == ESP_OK) printf("Started: %s\r\n", prof.name);
                else             printf("Error: %s\r\n", esp_err_to_name(e));
            } else {
                printf("Profile not found: '%s'\r\n", line + 6);
            }
        } else if (strcmp(line, "stop") == 0) {
            reflow_ctrl_stop();
            printf("Stopped\r\n");
        } else if (strcmp(line, "status") == 0) {
            ctrl_status_t st;
            reflow_ctrl_get_status(&st);
            static const char *state_names[] = {"idle", "running", "complete", "error"};
            printf("state=%s temp=%.1f setpoint=%.1f elapsed=%ds phase=%s duty=%d/4\r\n",
                   state_names[st.state], (double)st.temp, (double)st.setpoint,
                   st.elapsed_s, st.phase, st.duty_steps);
        } else if (strcmp(line, "wifi") == 0) {
            char ip[20] = {};
            wifi_manager_get_ip(ip, sizeof(ip));
            printf("mode=%s ip=%s\r\n",
                   wifi_manager_get_mode() == WIFI_MGR_STA ? "sta" : "ap", ip);
        } else if (strcmp(line, "wifi-clear") == 0) {
            wifi_manager_set_credentials("", "");
            printf("Wi-Fi credentials cleared — rebooting into AP mode\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        } else if (strcmp(line, "wifi-scan") == 0) {
            char *buf = malloc(2048);
            if (buf) {
                wifi_manager_scan_json(buf, 2048);
                printf("%s\r\n", buf);
                free(buf);
            }
        } else {
            printf("%s", HELP);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* NVS must be initialised before WiFi and Zigbee */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* TCP/IP stack and event loop must be created before any component that
       uses them (Wi-Fi, Zigbee coexistence layer, etc.).  esp_netif_init() is
       idempotent; esp_event_loop_create_default() returns ESP_ERR_INVALID_STATE
       if something already created it — treat that as success. */
    ESP_ERROR_CHECK(esp_netif_init());
    {
        esp_err_t e = esp_event_loop_create_default();
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(e);
    }

    ESP_LOGI(TAG, "Mounting SPIFFS...");
    ESP_ERROR_CHECK(profile_init());

    ESP_LOGI(TAG, "Initialising temperature sensor...");
    if (temperature_init() != ESP_OK) {
        ESP_LOGW(TAG, "MLX90614 init failed — temperature reads will fail");
    }

    ESP_LOGI(TAG, "Initialising reflow controller...");
    ESP_ERROR_CHECK(reflow_ctrl_init());

    ESP_LOGI(TAG, "Initialising Wi-Fi...");
    ESP_ERROR_CHECK(wifi_manager_init());

    char ip[20] = {0};
    if (wifi_manager_get_mode() == WIFI_MGR_STA) {
        wifi_manager_get_ip(ip, sizeof(ip));
        ESP_LOGI(TAG, "Wi-Fi connected — http://%s", ip);
    } else {
        ESP_LOGI(TAG, "AP mode — connect to 'Reflow-Setup' (open, no password) then open http://192.168.4.1");
    }

    ESP_LOGI(TAG, "Starting web server...");
    ESP_ERROR_CHECK(web_server_start());

    /* Zigbee only runs in STA mode — SoftAP + 802.15.4 share one antenna
       and the ESP32-C6 coexistence arbiter does not support that combination.
       In AP mode the device is in setup-only state; Zigbee starts after the
       user saves Wi-Fi credentials and the device reboots into STA mode. */
    if (wifi_manager_get_mode() == WIFI_MGR_STA) {
        ESP_LOGI(TAG, "Initialising Zigbee...");
        ESP_ERROR_CHECK(esp_coex_wifi_i154_enable());
        ESP_ERROR_CHECK(zigbee_plug_init());
    } else {
        ESP_LOGI(TAG, "AP (setup) mode — Zigbee skipped");
    }

    ESP_LOGI(TAG, "Ready. %d profile(s) loaded.", profile_count());

    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_vfs_dev_use_driver(UART_NUM_0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);

    xTaskCreate(console_task, "console", 4096, NULL, 3, NULL);
}
