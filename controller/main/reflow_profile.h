#pragma once
#include "esp_err.h"
#include <stdbool.h>

#define PROFILE_MAX_WAYPOINTS  16
#define PROFILE_NAME_LEN       64
#define PROFILE_DESC_LEN       128
#define PROFILE_LABEL_LEN      32
#define PROFILE_MAX_COUNT      10

typedef struct {
    int   time_s;
    float temp;
    char  label[PROFILE_LABEL_LEN];
} waypoint_t;

typedef struct {
    char       name[PROFILE_NAME_LEN];
    char       description[PROFILE_DESC_LEN];
    waypoint_t waypoints[PROFILE_MAX_WAYPOINTS];
    int        num_waypoints;
} reflow_profile_t;

esp_err_t profile_init(void);
int       profile_count(void);
esp_err_t profile_get(int index, reflow_profile_t *out);
esp_err_t profile_get_by_name(const char *name, reflow_profile_t *out);
esp_err_t profile_save(const char *name, const char *json_str);
esp_err_t profile_delete(const char *name);
esp_err_t profile_list_json(char *buf, size_t buf_len);

float       profile_interpolate_setpoint(const reflow_profile_t *p, int elapsed_s);
int         profile_total_duration(const reflow_profile_t *p);
const char *profile_phase_at(const reflow_profile_t *p, int elapsed_s);
bool        profile_is_cooling_at(const reflow_profile_t *p, int elapsed_s);
