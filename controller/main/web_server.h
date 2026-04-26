#pragma once
#include "esp_err.h"

esp_err_t web_server_start(void);
esp_err_t web_server_stop(void);
void      web_server_broadcast(const char *json);
