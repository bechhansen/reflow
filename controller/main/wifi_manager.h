#pragma once
#include "esp_err.h"
#include <stdbool.h>

typedef enum {
    WIFI_MGR_DISCONNECTED,
    WIFI_MGR_STA,
    WIFI_MGR_AP,
} wifi_mgr_mode_t;

esp_err_t       wifi_manager_init(void);
wifi_mgr_mode_t wifi_manager_get_mode(void);
esp_err_t       wifi_manager_get_ip(char *buf, size_t len);
esp_err_t       wifi_manager_set_credentials(const char *ssid, const char *password);
esp_err_t       wifi_manager_scan_json(char *buf, size_t buf_len);
esp_err_t       wifi_manager_get_hostname(char *buf, size_t len);
esp_err_t       wifi_manager_set_hostname(const char *hostname);
esp_err_t       wifi_manager_get_ssid(char *buf, size_t len);
