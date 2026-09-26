#pragma once
/* Profile geometry: the waypoint table shared by profile storage, the web API
   and the control law, plus the pure maths over it. Deliberately free of
   ESP-IDF headers so it can be host-tested. reflow_profile.h adds the
   SPIFFS-backed storage API on top of this. */
#include <stdbool.h>

#define PROFILE_MAX_WAYPOINTS  16
#define PROFILE_NAME_LEN       64
#define PROFILE_DESC_LEN       128
#define PROFILE_LABEL_LEN      32

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

/* At least two waypoints, times strictly increasing from >= 0. */
bool  curve_valid(const reflow_profile_t *p);

/* Linear interpolation; clamped to the first/last waypoint outside the table. */
float curve_temp_at(const reflow_profile_t *p, float t);

/* Slope (°C/s) of the segment containing t; 0 outside the table. */
float curve_slope_at(const reflow_profile_t *p, float t);

/* Index of the waypoint that ENDS the segment containing t (its label names
   the phase being approached), clamped to [1, n-1]. */
int   curve_segment_at(const reflow_profile_t *p, float t);

float curve_duration(const reflow_profile_t *p);

/* Highest waypoint temperature in [t0, t1], including the curve at both ends. */
float curve_max_between(const reflow_profile_t *p, float t0, float t1);

/* Start of the cool-down: the time of the last waypoint after which the curve
   never rises again. The curve from here on only falls (or stays flat). */
float curve_cool_start(const reflow_profile_t *p);
