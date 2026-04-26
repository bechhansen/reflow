#pragma once
#include "esp_err.h"

esp_err_t temperature_init(void);
esp_err_t temperature_read(float *celsius);
