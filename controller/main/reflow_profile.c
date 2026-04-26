#include "reflow_profile.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <dirent.h>
#include <sys/stat.h>

#define PROFILE_DIR   "/spiffs/profiles"
#define MAX_JSON_SIZE 4096

static const char *TAG = "profile";

static esp_err_t parse_profile_json(const char *json_str, reflow_profile_t *out)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return ESP_ERR_INVALID_ARG;

    cJSON *name = cJSON_GetObjectItem(root, "name");
    cJSON *desc = cJSON_GetObjectItem(root, "description");
    cJSON *wps  = cJSON_GetObjectItem(root, "waypoints");

    if (!cJSON_IsString(name) || !cJSON_IsArray(wps)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    strlcpy(out->name, name->valuestring, sizeof(out->name));
    if (cJSON_IsString(desc)) strlcpy(out->description, desc->valuestring, sizeof(out->description));
    else out->description[0] = '\0';

    int n = cJSON_GetArraySize(wps);
    if (n > PROFILE_MAX_WAYPOINTS) n = PROFILE_MAX_WAYPOINTS;
    out->num_waypoints = n;

    for (int i = 0; i < n; i++) {
        cJSON *wp    = cJSON_GetArrayItem(wps, i);
        cJSON *t     = cJSON_GetObjectItem(wp, "time");
        cJSON *temp  = cJSON_GetObjectItem(wp, "temp");
        cJSON *label = cJSON_GetObjectItem(wp, "label");

        out->waypoints[i].time_s = cJSON_IsNumber(t) ? (int)t->valuedouble : 0;
        out->waypoints[i].temp   = cJSON_IsNumber(temp) ? (float)temp->valuedouble : 0.0f;
        if (cJSON_IsString(label)) strlcpy(out->waypoints[i].label, label->valuestring, PROFILE_LABEL_LEN);
        else out->waypoints[i].label[0] = '\0';
    }

    cJSON_Delete(root);
    return ESP_OK;
}

static char s_profile_path[128];

static const char *profile_path(const char *name)
{
    snprintf(s_profile_path, sizeof(s_profile_path), "%s/%s.json", PROFILE_DIR, name);
    return s_profile_path;
}

esp_err_t profile_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = "/spiffs",
        .partition_label        = "spiffs",
        .max_files              = 20,
        .format_if_mount_failed = true,
    };
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return err;
    }

    size_t total = 0, used = 0;
    esp_spiffs_info("spiffs", &total, &used);
    ESP_LOGI(TAG, "SPIFFS: %d/%d bytes used", used, total);
    return ESP_OK;
}

int profile_count(void)
{
    DIR *dir = opendir(PROFILE_DIR);
    if (!dir) return 0;
    int count = 0;
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        if (strstr(ent->d_name, ".json")) count++;
    }
    closedir(dir);
    return count;
}

esp_err_t profile_get(int index, reflow_profile_t *out)
{
    DIR *dir = opendir(PROFILE_DIR);
    if (!dir) return ESP_ERR_NOT_FOUND;

    int i = 0;
    struct dirent *ent;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    while ((ent = readdir(dir)) != NULL) {
        if (!strstr(ent->d_name, ".json")) continue;
        if (i == index) {
            char path[256];
            snprintf(path, sizeof(path), "%s/%.200s", PROFILE_DIR, ent->d_name);
            FILE *f = fopen(path, "r");
            if (f) {
                char *buf = malloc(MAX_JSON_SIZE);
                if (buf) {
                    size_t n = fread(buf, 1, MAX_JSON_SIZE - 1, f);
                    buf[n] = '\0';
                    err = parse_profile_json(buf, out);
                    free(buf);
                }
                fclose(f);
            }
            break;
        }
        i++;
    }
    closedir(dir);
    return err;
}

esp_err_t profile_get_by_name(const char *name, reflow_profile_t *out)
{
    FILE *f = fopen(profile_path(name), "r");
    if (!f) return ESP_ERR_NOT_FOUND;

    char *buf = malloc(MAX_JSON_SIZE);
    if (!buf) { fclose(f); return ESP_ERR_NO_MEM; }

    size_t n = fread(buf, 1, MAX_JSON_SIZE - 1, f);
    buf[n] = '\0';
    fclose(f);

    esp_err_t err = parse_profile_json(buf, out);
    free(buf);
    return err;
}

esp_err_t profile_save(const char *name, const char *json_str)
{
    reflow_profile_t tmp;
    esp_err_t err = parse_profile_json(json_str, &tmp);
    if (err != ESP_OK) return err;

    FILE *f = fopen(profile_path(name), "w");
    if (!f) return ESP_FAIL;
    fputs(json_str, f);
    fclose(f);
    ESP_LOGI(TAG, "Saved profile: %s", name);
    return ESP_OK;
}

esp_err_t profile_delete(const char *name)
{
    if (remove(profile_path(name)) != 0) return ESP_ERR_NOT_FOUND;
    ESP_LOGI(TAG, "Deleted profile: %s", name);
    return ESP_OK;
}

esp_err_t profile_list_json(char *buf, size_t buf_len)
{
    cJSON *arr = cJSON_CreateArray();
    DIR *dir = opendir(PROFILE_DIR);
    if (dir) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            if (!strstr(ent->d_name, ".json")) continue;
            char name[PROFILE_NAME_LEN];
            strlcpy(name, ent->d_name, sizeof(name));
            char *dot = strrchr(name, '.');
            if (dot) *dot = '\0';
            cJSON_AddItemToArray(arr, cJSON_CreateString(name));
        }
        closedir(dir);
    }
    char *str = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if (!str) return ESP_ERR_NO_MEM;
    strlcpy(buf, str, buf_len);
    free(str);
    return ESP_OK;
}

float profile_interpolate_setpoint(const reflow_profile_t *p, int elapsed_s)
{
    if (p->num_waypoints == 0) return 0.0f;
    if (elapsed_s <= p->waypoints[0].time_s) return p->waypoints[0].temp;

    int last = p->num_waypoints - 1;
    if (elapsed_s >= p->waypoints[last].time_s) return p->waypoints[last].temp;

    for (int i = 0; i < last; i++) {
        if (elapsed_s >= p->waypoints[i].time_s && elapsed_s < p->waypoints[i + 1].time_s) {
            float span = (float)(p->waypoints[i + 1].time_s - p->waypoints[i].time_s);
            float t    = (float)(elapsed_s - p->waypoints[i].time_s) / span;
            return p->waypoints[i].temp + t * (p->waypoints[i + 1].temp - p->waypoints[i].temp);
        }
    }
    return p->waypoints[last].temp;
}

int profile_total_duration(const reflow_profile_t *p)
{
    if (p->num_waypoints == 0) return 0;
    return p->waypoints[p->num_waypoints - 1].time_s;
}

const char *profile_phase_at(const reflow_profile_t *p, int elapsed_s)
{
    if (p->num_waypoints == 0) return "—";
    if (elapsed_s >= profile_total_duration(p)) return p->waypoints[p->num_waypoints - 1].label;

    for (int i = 0; i < p->num_waypoints - 1; i++) {
        if (elapsed_s >= p->waypoints[i].time_s && elapsed_s < p->waypoints[i + 1].time_s)
            return p->waypoints[i + 1].label;
    }
    return p->waypoints[0].label;
}

bool profile_is_cooling_at(const reflow_profile_t *p, int elapsed_s)
{
    if (p->num_waypoints < 2) return false;
    int last = p->num_waypoints - 1;
    if (elapsed_s >= p->waypoints[last].time_s) return true; /* past last waypoint */
    for (int i = 0; i < last; i++) {
        if (elapsed_s >= p->waypoints[i].time_s && elapsed_s < p->waypoints[i + 1].time_s)
            return p->waypoints[i + 1].temp < p->waypoints[i].temp;
    }
    return false;
}
