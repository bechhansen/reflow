#include "reflow_curve.h"

bool curve_valid(const reflow_profile_t *p)
{
    if (p->num_waypoints < 2 || p->num_waypoints > PROFILE_MAX_WAYPOINTS) return false;
    if (p->waypoints[0].time_s < 0) return false;
    for (int i = 1; i < p->num_waypoints; i++)
        if (p->waypoints[i].time_s <= p->waypoints[i - 1].time_s) return false;
    return true;
}

int curve_segment_at(const reflow_profile_t *p, float t)
{
    int n = p->num_waypoints;
    for (int i = 1; i < n - 1; i++)
        if (t < (float)p->waypoints[i].time_s) return i;
    return n - 1;
}

float curve_temp_at(const reflow_profile_t *p, float t)
{
    const waypoint_t *w = p->waypoints;
    int n = p->num_waypoints;
    if (t <= (float)w[0].time_s)     return w[0].temp;
    if (t >= (float)w[n - 1].time_s) return w[n - 1].temp;
    int i = curve_segment_at(p, t);
    float t0 = (float)w[i - 1].time_s, t1 = (float)w[i].time_s;
    return w[i - 1].temp + (w[i].temp - w[i - 1].temp) * (t - t0) / (t1 - t0);
}

float curve_slope_at(const reflow_profile_t *p, float t)
{
    const waypoint_t *w = p->waypoints;
    int n = p->num_waypoints;
    if (t < (float)w[0].time_s || t >= (float)w[n - 1].time_s) return 0.0f;
    int i = curve_segment_at(p, t);
    return (w[i].temp - w[i - 1].temp) / (float)(w[i].time_s - w[i - 1].time_s);
}

float curve_duration(const reflow_profile_t *p)
{
    return (float)p->waypoints[p->num_waypoints - 1].time_s;
}

float curve_max_between(const reflow_profile_t *p, float t0, float t1)
{
    float a = curve_temp_at(p, t0), b = curve_temp_at(p, t1);
    float m = a > b ? a : b;
    for (int i = 0; i < p->num_waypoints; i++) {
        float t = (float)p->waypoints[i].time_s;
        if (t > t0 && t < t1 && p->waypoints[i].temp > m) m = p->waypoints[i].temp;
    }
    return m;
}

float curve_cool_start(const reflow_profile_t *p)
{
    int i = p->num_waypoints - 1;
    while (i > 0 && p->waypoints[i - 1].temp > p->waypoints[i].temp) i--;
    return (float)p->waypoints[i].time_s;
}
