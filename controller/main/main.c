#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_coexist.h"

#include "temperature.h"
#include "zigbee_plug.h"
#include "plug_ctrl.h"
#include "reflow_profile.h"
#include "reflow_ctrl.h"
#include "wifi_manager.h"
#include "web_server.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "main";

/* ── Console ─────────────────────────────────────────────────────────────── */

static const char *HELP =
    "Commands: start <profile>, step <duty> <secs> <maxT>, hold <temp>, stop, status,\r\n"
    "  ctl show|save|reset, ctl set <key> <val>, log on|off,\r\n"
    "  plug on|off|toggle|status, wifi, wifi-scan, wifi-clear\r\n";

static SemaphoreHandle_t s_console_mutex;   /* two inputs, one command at a time */

static void handle_line(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) line[--len] = '\0';
    if (len == 0) return;

    const char *why = NULL;
    float a1, a2, a3;
    char  key[24];
    if (strncmp(line, "start ", 6) == 0) {
        reflow_profile_t prof;
        if (profile_get_by_name(line + 6, &prof) == ESP_OK) {
            if (reflow_ctrl_start(&prof, &why) == ESP_OK) printf("Started: %s\r\n", prof.name);
            else                                          printf("Refused: %s\r\n", why);
        } else {
            printf("Profile not found: '%s'\r\n", line + 6);
        }
    } else if (sscanf(line, "step %f %f %f", &a1, &a2, &a3) == 3) {
        if (reflow_ctrl_start_step(a1, a2, a3, &why) == ESP_OK) printf("Started: step\r\n");
        else                                                    printf("Refused: %s\r\n", why);
    } else if (sscanf(line, "hold %f", &a1) == 1) {
        if (reflow_ctrl_start_hold(a1, &why) == ESP_OK) printf("Started: hold\r\n");
        else                                            printf("Refused: %s\r\n", why);
    } else if (strcmp(line, "stop") == 0) {
        reflow_ctrl_stop();
        printf("Stopped\r\n");
    } else if (strcmp(line, "status") == 0) {
        ctrl_status_t st;
        reflow_ctrl_get_status(&st);
        printf("state=%s temp=%.1f ambient=%.1f sensor=%s",
               reflow_ctrl_state_str(st.state), (double)st.temp, (double)st.ambient,
               st.sensor_ok ? "ok" : "fault");
        if (st.state != CTRL_STATE_IDLE)
            printf(" run=%s elapsed=%.0f pt=%.0f sp=%.1f power=%.2f phase=%s fault=%s",
                   st.profile, (double)st.elapsed, (double)st.profile_t, (double)st.setpoint,
                   (double)st.power, st.phase, st.fault);
        printf("\r\n");
    } else if (strcmp(line, "ctl show") == 0) {
        reflow_ctrl_params_print();
    } else if (strcmp(line, "ctl save") == 0) {
        esp_err_t e = reflow_ctrl_params_save();
        if (e == ESP_OK) printf("Saved\r\n");
        else             printf("Save failed: %s\r\n", esp_err_to_name(e));
    } else if (strcmp(line, "ctl reset") == 0) {
        reflow_ctrl_params_reset();
        printf("Defaults restored\r\n");
    } else if (sscanf(line, "ctl set %23s %f", key, &a1) == 2) {
        if (reflow_ctrl_param_set(key, a1) == ESP_OK) printf("%s=%g\r\n", key, (double)a1);
        else                                          printf("Unknown parameter '%s'\r\n", key);
    } else if (strcmp(line, "log on") == 0 || strcmp(line, "log off") == 0) {
        reflow_ctrl_set_log(strcmp(line, "log on") == 0);
        printf("OK\r\n");
    } else if (strcmp(line, "plug status") == 0) {
        plug_status_t ps;
        plug_ctrl_get_status(&ps);
        printf("available=%s state=%s pending=%s\r\n",
               ps.available ? "yes" : "no", plug_state_str(ps.state),
               ps.pending ? (ps.target ? "->on" : "->off") : "no");
    } else if (strcmp(line, "plug on") == 0 || strcmp(line, "plug off") == 0 ||
               strcmp(line, "plug toggle") == 0) {
        if (reflow_ctrl_is_active()) { printf("Rejected: a run owns the plug\r\n"); return; }
        esp_err_t e = strcmp(line, "plug toggle") == 0 ? plug_ctrl_toggle()
                    : plug_ctrl_set(strcmp(line, "plug on") == 0);
        if (e == ESP_OK) printf("Requested; awaiting confirmation\r\n");
        else             printf("Rejected: %s\r\n", plug_ctrl_err_reason(e));
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

static void run_line(char *line)
{
    xSemaphoreTake(s_console_mutex, portMAX_DELAY);
    handle_line(line);
    xSemaphoreGive(s_console_mutex);
}

/* UART0 (GPIO16/17): the primary console. */
static void uart_console_task(void *arg)
{
    char line[128];
    printf("\r\nReflow console ready. %s", HELP);
    while (fgets(line, sizeof(line), stdin)) run_line(line);
    vTaskDelete(NULL);
}

/* The chip's USB-Serial-JTAG port, where the logs are mirrored as the
   secondary console. stdin is UART0 only, so commands arriving here are read
   through the driver. Output still goes out through the (non-driver)
   secondary console, so printf never blocks when no USB host is listening. */
static void usb_console_task(void *arg)
{
    char   line[128];
    size_t len = 0;
    for (;;) {
        char c;
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) continue;
        if (c == '\r' || c == '\n') {
            line[len] = '\0';
            if (len) run_line(line);
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
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

    /* Before the controller and the web server (which subscribe to plug
       results) and Zigbee (which delivers the plug events). */
    ESP_ERROR_CHECK(plug_ctrl_init());

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

    s_console_mutex = xSemaphoreCreateMutex();
    xTaskCreate(uart_console_task, "console", 4096, NULL, 3, NULL);

    usb_serial_jtag_driver_config_t usj = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&usj) == ESP_OK)
        xTaskCreate(usb_console_task, "usb_console", 4096, NULL, 3, NULL);
    else
        ESP_LOGW(TAG, "USB-Serial-JTAG driver install failed — USB console input disabled");
}
