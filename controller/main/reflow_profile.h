#pragma once
#include "esp_err.h"
#include <stdbool.h>
/* Waypoint table, the profile struct and the pure curve maths live in
   reflow_curve.h so that the control law and its host tests can share them
   without pulling in ESP-IDF. This header adds SPIFFS-backed storage. */
#include "reflow_curve.h"

esp_err_t profile_init(void);
int       profile_count(void);
esp_err_t profile_get(int index, reflow_profile_t *out);
esp_err_t profile_get_by_name(const char *name, reflow_profile_t *out);
esp_err_t profile_save(const char *name, const char *json_str);
esp_err_t profile_delete(const char *name);
esp_err_t profile_list_json(char *buf, size_t buf_len);
