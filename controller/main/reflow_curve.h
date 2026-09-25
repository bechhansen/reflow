#pragma once
/* Profile geometry: the waypoint table shared by profile storage and the
   web API. Deliberately free of ESP-IDF headers. reflow_profile.h adds the
   SPIFFS-backed storage API on top of this.

   The pure maths over the table (setpoint interpolation, phase lookup,
   slope) went with the control law that was its only caller. */

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
