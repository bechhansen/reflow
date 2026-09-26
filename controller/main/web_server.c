#include "web_server.h"
#include "reflow_ctrl.h"
#include "reflow_profile.h"
#include "zigbee_plug.h"
#include "plug_ctrl.h"
#include "wifi_manager.h"
#include "ota_update.h"
#include "web_files.h"
#include "esp_app_desc.h"
#include "sdkconfig.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include "esp_spiffs.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>

#define MAX_WS_CLIENTS   4
/* Must match cfg.max_open_sockets below: httpd_get_client_list() needs an array
   sized for every socket the server may hold open. */
#define HTTPD_MAX_OPEN_SOCKETS 10
#define MAX_RESP_BUF     4096
#define TELEMETRY_HZ     2
#define PAIR_DURATION_S  60

static const char *TAG = "web_server";

static httpd_handle_t   s_server = NULL;
static int              s_ws_fds[MAX_WS_CLIENTS];
static SemaphoreHandle_t s_ws_mutex;
static esp_timer_handle_t s_telemetry_timer;
static char             s_temp_unit = 'C';   /* display unit, 'C' or 'F' (see Settings) */

/* ── WebSocket client list ───────────────────────────────────────────────── */

static void ws_client_add(int fd)
{
    int slot = -1, count = 0;
    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == -1 && slot == -1) { s_ws_fds[i] = fd; slot = i; }
    }
    for (int i = 0; i < MAX_WS_CLIENTS; i++) if (s_ws_fds[i] != -1) count++;
    xSemaphoreGive(s_ws_mutex);
    if (slot == -1) {
        ESP_LOGW(TAG, "WS client fd=%d rejected — all %d slots busy", fd, MAX_WS_CLIENTS);
    } else {
        ESP_LOGI(TAG, "WS client connected fd=%d (slot %d, %d active)", fd, slot, count);
    }
}

static void ws_client_remove(int fd)
{
    bool found = false;
    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == fd) { s_ws_fds[i] = -1; found = true; break; }
    }
    xSemaphoreGive(s_ws_mutex);
    if (found) ESP_LOGI(TAG, "WS client disconnected fd=%d", fd);
}

void web_server_broadcast(const char *json)
{
    if (!s_server) return;

    httpd_ws_frame_t frame = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json,
        .len     = strlen(json),
    };

    /* Enumerate httpd's own socket list rather than a list we maintain ourselves.
       Under ESP-IDF 6.x the URI handler is NOT invoked for the WebSocket
       handshake GET (that is now gated behind the opt-in pre/post-handshake
       callbacks), so ws_client_add() never fires and a self-managed fd table
       stays empty — telemetry then goes to nobody while the handshake still
       succeeds. httpd_ws_get_fd_info() tells us which live sockets are
       WebSockets, which works regardless of how the handshake was dispatched. */
    size_t fd_count = HTTPD_MAX_OPEN_SOCKETS;
    int    fds[HTTPD_MAX_OPEN_SOCKETS];

    if (httpd_get_client_list(s_server, &fd_count, fds) != ESP_OK) return;

    for (size_t i = 0; i < fd_count; i++) {
        if (httpd_ws_get_fd_info(s_server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        esp_err_t err = httpd_ws_send_frame_async(s_server, fds[i], &frame);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "WS client %d send failed (%s), closing", fds[i], esp_err_to_name(err));
            httpd_sess_trigger_close(s_server, fds[i]);
        }
    }
}

/* ── Telemetry ───────────────────────────────────────────────────────────── */

static void telemetry_cb(void *arg)
{
    ctrl_status_t st;
    reflow_ctrl_get_status(&st);

    char ip[20] = "0.0.0.0";
    wifi_manager_get_ip(ip, sizeof(ip));

    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type",        "telemetry");
    cJSON_AddNumberToObject(obj, "temp",        (double)st.temp);
    cJSON_AddNumberToObject(obj, "ambient",     st.ambient);
    cJSON_AddStringToObject(obj, "state",       reflow_ctrl_state_str(st.state));
    cJSON_AddBoolToObject  (obj, "sensor_ok",   st.sensor_ok);
    cJSON_AddBoolToObject  (obj, "plug_paired", zigbee_plug_is_paired());
    {
        /* Confirmed plug state from the plug service: "unknown" until a read
           has answered, and a pending change is not reported as done. */
        plug_status_t ps;
        plug_ctrl_get_status(&ps);
        cJSON_AddBoolToObject  (obj, "plug_available", ps.available);
        cJSON_AddStringToObject(obj, "plug_state",     plug_state_str(ps.state));
        cJSON_AddBoolToObject  (obj, "plug_pending",   ps.pending);
    }
    if (st.state == CTRL_STATE_IDLE) {
        cJSON_AddStringToObject(obj, "profile", "");
    } else {
        cJSON_AddStringToObject(obj, "profile",   st.profile);
        cJSON_AddStringToObject(obj, "phase",     st.phase);
        cJSON_AddNumberToObject(obj, "elapsed",   (double)st.elapsed);
        cJSON_AddNumberToObject(obj, "profile_t", (double)st.profile_t);
        cJSON_AddNumberToObject(obj, "setpoint",  (double)st.setpoint);
        cJSON_AddNumberToObject(obj, "power",     (double)st.power);
        cJSON_AddStringToObject(obj, "fault",     st.fault);
        cJSON_AddNumberToObject(obj, "run_id",    st.run_id);
    }
    {
        char host[40];
        wifi_manager_get_hostname(host, sizeof(host));
        cJSON_AddStringToObject(obj, "hostname", host);
    }
    {
        char unit[2] = { s_temp_unit, 0 };
        cJSON_AddStringToObject(obj, "temp_unit", unit);
    }
    {
        ota_status_t os;
        ota_update_get_status(&os);
        cJSON_AddStringToObject(obj, "ota_state", ota_state_str(os.state));
        if (os.state == OTA_AVAILABLE) cJSON_AddStringToObject(obj, "ota_latest", os.latest);
        if (os.state == OTA_INSTALLING) cJSON_AddNumberToObject(obj, "ota_progress", os.progress);
    }
    cJSON_AddStringToObject(obj, "wifi_mode",
                            wifi_manager_get_mode() == WIFI_MGR_STA ? "sta" : "ap");
    cJSON_AddStringToObject(obj, "ip", ip);

    char *str = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (str) { web_server_broadcast(str); free(str); }
}

/* ── Pairing callback (called from Zigbee task) ─────────────────────────── */

static void pairing_result_cb(bool success, uint16_t addr, uint8_t ep)
{
    cJSON *obj = cJSON_CreateObject();
    if (success) {
        char name[32];
        snprintf(name, sizeof(name), "plug_%04x", addr);
        cJSON_AddStringToObject(obj, "type",      "pairing_success");
        cJSON_AddStringToObject(obj, "plug_name", name);
    } else {
        cJSON_AddStringToObject(obj, "type", "pairing_failed");
    }
    char *str = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (str) { web_server_broadcast(str); free(str); }
}

/* ── Plug result callback (called from Zigbee task) ─────────────────────── */

static void plug_result_cb(bool ok, plug_state_t state, const char *reason)
{
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type",  "plug_result");
    cJSON_AddBoolToObject  (obj, "ok",    ok);
    cJSON_AddStringToObject(obj, "state", plug_state_str(state));
    if (reason) cJSON_AddStringToObject(obj, "reason", reason);
    char *str = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (str) { web_server_broadcast(str); free(str); }
}

/* A rejected request goes back to the client that asked, not to everyone. */
static void ws_reply_plug_rejected(httpd_req_t *req, esp_err_t err)
{
    char buf[96];
    plug_status_t ps;
    plug_ctrl_get_status(&ps);
    int n = snprintf(buf, sizeof(buf),
                     "{\"type\":\"plug_result\",\"ok\":false,\"state\":\"%s\",\"reason\":\"%s\"}",
                     plug_state_str(ps.state), plug_ctrl_err_reason(err));
    httpd_ws_frame_t f = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)buf,
        .len     = (size_t)n,
    };
    httpd_ws_send_frame(req, &f);
}

/* A refused start goes back to the client that asked. */
static void ws_reply_run_refused(httpd_req_t *req, const char *reason)
{
    char buf[80];
    int n = snprintf(buf, sizeof(buf), "{\"type\":\"run_result\",\"ok\":false,\"reason\":\"%s\"}",
                     reason);
    httpd_ws_frame_t f = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)buf,
        .len     = (size_t)n,
    };
    httpd_ws_send_frame(req, &f);
}

static void pairing_countdown_cb(uint8_t remaining)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"type\":\"pairing_open\",\"remaining\":%d}", remaining);
    web_server_broadcast(buf);
}

/* ── WebSocket handler ───────────────────────────────────────────────────── */

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* New connection */
        ws_client_add(httpd_req_to_sockfd(req));
        ESP_LOGD(TAG, "WS client connected: %d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    /* Step 1: read header — frame.len must be 0 so the function parses the
       WebSocket frame header and stores the actual payload length in frame.len */
    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        ws_client_remove(httpd_req_to_sockfd(req));
        return err;
    }
    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ws_client_remove(httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    /* Step 2: read payload now that we know its exact length */
    uint8_t buf[512] = {0};
    if (frame.len > 0) {
        if (frame.len >= sizeof(buf)) {
            ws_client_remove(httpd_req_to_sockfd(req));
            return ESP_ERR_INVALID_SIZE;
        }
        frame.payload = buf;
        err = httpd_ws_recv_frame(req, &frame, frame.len);
        if (err != ESP_OK) {
            ws_client_remove(httpd_req_to_sockfd(req));
            return err;
        }
    }

    buf[frame.len] = '\0';
    cJSON *msg = cJSON_Parse((char *)buf);
    if (!msg) return ESP_OK;

    cJSON *type = cJSON_GetObjectItem(msg, "type");
    if (!cJSON_IsString(type)) { cJSON_Delete(msg); return ESP_OK; }

    const char *t = type->valuestring;

    if (strcmp(t, "start") == 0) {
        cJSON *pname = cJSON_GetObjectItem(msg, "profile");
        reflow_profile_t profile;
        const char *why = "not_found";
        if (!cJSON_IsString(pname) ||
            profile_get_by_name(pname->valuestring, &profile) != ESP_OK ||
            reflow_ctrl_start(&profile, &why) != ESP_OK)
            ws_reply_run_refused(req, why);
    } else if (strcmp(t, "stop") == 0) {
        reflow_ctrl_stop();
    } else if (strcmp(t, "start_pairing") == 0) {
        zigbee_plug_start_pairing(PAIR_DURATION_S, pairing_result_cb, pairing_countdown_cb);
    } else if (strcmp(t, "unpair") == 0) {
        zigbee_plug_unpair();
    } else if (strcmp(t, "plug_toggle") == 0) {
        /* During a run the controller owns the plug. */
        esp_err_t perr = reflow_ctrl_is_active() ? ESP_ERR_NOT_ALLOWED : plug_ctrl_toggle();
        if (perr != ESP_OK) ws_reply_plug_rejected(req, perr);
    }

    cJSON_Delete(msg);
    return ESP_OK;
}

/* ── REST: profile list ──────────────────────────────────────────────────── */

static esp_err_t api_profiles_get(httpd_req_t *req)
{
    char *buf = malloc(MAX_RESP_BUF);
    if (!buf) return httpd_resp_send_500(req);
    profile_list_json(buf, MAX_RESP_BUF);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    free(buf);
    return ESP_OK;
}

/* ── REST: run trace ─────────────────────────────────────────────────────── */

/* {"run_id":N,"points":[[elapsed,temp,setpoint,heater,power],...]}: the
   current or last run's trace, so a page opened mid-run can draw the whole of
   it. heater is the confirmed state (1 on, 0 off, -1 unknown), power the
   requested duty 0..1. Streamed in chunks; the trace can hold ~1200 points. */
static esp_err_t api_run_trace_get(httpd_req_t *req)
{
    enum { CHUNK = 64 };
    /* Static: ~2 KB would crowd the httpd task stack, and handlers run one at
       a time in that task. */
    static ctrl_trace_pt_t pts[CHUNK];
    static char  buf[CHUNK * 40 + 32];
    ctrl_status_t st;
    reflow_ctrl_get_status(&st);

    httpd_resp_set_type(req, "application/json");
    snprintf(buf, sizeof(buf), "{\"run_id\":%lu,\"points\":[", (unsigned long)st.run_id);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    int first = 0, n;
    while ((n = reflow_ctrl_trace_get(first, pts, CHUNK)) > 0) {
        int len = 0;
        for (int k = 0; k < n; k++)
            len += snprintf(buf + len, sizeof(buf) - len, "%s[%.1f,%.1f,%.1f,%d,%.2f]",
                            first + k ? "," : "", (double)pts[k].t, (double)pts[k].temp,
                            (double)pts[k].sp, pts[k].plug, (double)pts[k].power);
        httpd_resp_send_chunk(req, buf, len);
        first += n;
    }
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* ── REST: single profile ────────────────────────────────────────────────── */

static const char *name_from_uri(const char *uri)
{
    const char *last_slash = strrchr(uri, '/');
    return last_slash ? last_slash + 1 : uri;
}

static void url_decode(const char *src, char *dst, size_t dst_len)
{
    size_t i = 0;
    while (*src && i + 1 < dst_len) {
        if (*src == '%' && src[1] && src[2]) {
            char hex[3] = { src[1], src[2], '\0' };
            dst[i++] = (char)strtol(hex, NULL, 16);
            src += 3;
        } else {
            dst[i++] = *src++;
        }
    }
    dst[i] = '\0';
}

static esp_err_t api_profile_get(httpd_req_t *req)
{
    char name[64];
    url_decode(name_from_uri(req->uri), name, sizeof(name));
    reflow_profile_t p;
    if (profile_get_by_name(name, &p) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_OK;
    }
    /* Re-read raw JSON from SPIFFS */
    char path[128];
    snprintf(path, sizeof(path), "/spiffs/profiles/%s.json", name);
    FILE *f = fopen(path, "r");
    if (!f) { httpd_resp_send_404(req); return ESP_OK; }
    char *buf = malloc(MAX_RESP_BUF);
    if (!buf) { fclose(f); return httpd_resp_send_500(req); }
    size_t n = fread(buf, 1, MAX_RESP_BUF - 1, f);
    buf[n] = '\0';
    fclose(f);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    free(buf);
    return ESP_OK;
}

static esp_err_t api_profile_post(httpd_req_t *req)
{
    char name[64];
    url_decode(name_from_uri(req->uri), name, sizeof(name));
    char *buf = malloc(MAX_RESP_BUF);
    if (!buf) return httpd_resp_send_500(req);

    int total = req->content_len;
    if (total >= MAX_RESP_BUF) { free(buf); return httpd_resp_send_500(req); }

    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) { free(buf); return ESP_FAIL; }
        received += r;
    }
    buf[received] = '\0';

    esp_err_t err = profile_save(name, buf);
    free(buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, err == ESP_OK ? "{\"ok\":true}" : "{\"ok\":false}");
    return ESP_OK;
}

static esp_err_t api_profile_delete(httpd_req_t *req)
{
    char name[64];
    url_decode(name_from_uri(req->uri), name, sizeof(name));
    esp_err_t err = profile_delete(name);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, err == ESP_OK ? "{\"ok\":true}" : "{\"ok\":false}");
    return ESP_OK;
}

/* ── Settings ──────────────────────────────────────────────────────────────
   Device-wide display settings, kept in NVS namespace "ui". The temperature
   unit only changes how the web pages show and take temperatures: firmware,
   stored profiles, console and logs are always °C. */

#define UI_NVS_NS "ui"

static void settings_load(void)
{
    nvs_handle_t h;
    uint8_t v = 'C';
    if (nvs_open(UI_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "temp_unit", &v);
        nvs_close(h);
    }
    s_temp_unit = (v == 'F') ? 'F' : 'C';
}

static esp_err_t api_settings_get(httpd_req_t *req)
{
    char buf[40];
    snprintf(buf, sizeof(buf), "{\"temp_unit\":\"%c\"}", s_temp_unit);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

/* {"temp_unit":"C"|"F"} */
static esp_err_t api_settings_post(httpd_req_t *req)
{
    char buf[128] = {0};
    int r = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (r <= 0) return ESP_FAIL;
    cJSON *msg = cJSON_Parse(buf);
    cJSON *u   = msg ? cJSON_GetObjectItem(msg, "temp_unit") : NULL;
    char unit  = 0;
    if (cJSON_IsString(u) && (strcmp(u->valuestring, "C") == 0 || strcmp(u->valuestring, "F") == 0))
        unit = u->valuestring[0];
    cJSON_Delete(msg);
    if (!unit) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "temp_unit must be \"C\" or \"F\"");
        return ESP_OK;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(UI_NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, "temp_unit", (uint8_t)unit);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err == ESP_OK) s_temp_unit = unit;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, err == ESP_OK ? "{\"ok\":true}" : "{\"ok\":false}");
}

/* ── REST: Wi-Fi ──────────────────────────────────────────────────────────── */

static esp_err_t api_wifi_scan(httpd_req_t *req)
{
    char *buf = malloc(2048);
    if (!buf) return httpd_resp_send_500(req);
    wifi_manager_scan_json(buf, 2048);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    free(buf);
    return ESP_OK;
}

static esp_err_t api_wifi_config_get(httpd_req_t *req)
{
    char ssid[64] = {0}, host[40] = {0}, ip[20] = {0};
    wifi_manager_get_ssid(ssid, sizeof(ssid));
    wifi_manager_get_hostname(host, sizeof(host));
    wifi_manager_get_ip(ip, sizeof(ip));

    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "ssid", ssid);
    cJSON_AddStringToObject(obj, "hostname", host);
    cJSON_AddStringToObject(obj, "mode",
                            wifi_manager_get_mode() == WIFI_MGR_STA ? "sta" : "ap");
    cJSON_AddStringToObject(obj, "ip", ip);

    char *str = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (!str) return httpd_resp_send_500(req);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, str);
    free(str);
    return ESP_OK;
}

static esp_err_t api_wifi_credentials_post(httpd_req_t *req)
{
    char buf[256] = {0};
    int r = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (r <= 0) return ESP_FAIL;
    buf[r] = '\0';

    cJSON *obj  = cJSON_Parse(buf);
    cJSON *ssid = cJSON_GetObjectItem(obj, "ssid");
    cJSON *pass = cJSON_GetObjectItem(obj, "password");
    cJSON *host = cJSON_GetObjectItem(obj, "hostname");

    /* Either field may be sent on its own: the settings page can change the
       hostname without re-entering Wi-Fi credentials, and vice versa. */
    bool did_ssid = false, did_host = false;
    const char *host_err = NULL;

    if (cJSON_IsString(ssid) && ssid->valuestring[0] != '\0') {
        wifi_manager_set_credentials(ssid->valuestring,
                                      cJSON_IsString(pass) ? pass->valuestring : "");
        did_ssid = true;
    }
    if (cJSON_IsString(host) && host->valuestring[0] != '\0') {
        if (wifi_manager_set_hostname(host->valuestring) == ESP_OK) did_host = true;
        else host_err = "invalid hostname: use a-z, 0-9 and hyphens, max 32 chars";
    }
    cJSON_Delete(obj);

    httpd_resp_set_type(req, "application/json");
    if (!did_ssid && !did_host) {
        char msg[160];
        snprintf(msg, sizeof(msg), "{\"ok\":false,\"error\":\"%s\"}",
                 host_err ? host_err : "nothing to save");
        httpd_resp_sendstr(req, msg);
        return ESP_OK;
    }
    httpd_resp_sendstr(req, "{\"ok\":true,\"rebooting\":true}");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* ── Captive portal ───────────────────────────────────────────────────────── */

static esp_err_t captive_redirect(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

/* Apple: expects this exact body to detect no-captive-portal */
static esp_err_t captive_apple(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
    return ESP_OK;
}

/* Android: expects 204 No Content */
static esp_err_t captive_generate_204(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

/* ── Static files (embedded in the firmware) ──────────────────────────────
   The web UI is part of the app image (web_files.c, gzipped), so an update
   carries it. Every file gets the firmware build hash as its ETag with
   no-cache: the browser revalidates each load and gets a 304 until the
   firmware changes, and never runs old UI code against new firmware. */

static char s_etag[24];

static esp_err_t static_file_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    if (strcmp(uri, "/") == 0) uri = "/index.html";
    size_t ulen = strcspn(uri, "?#");
    const embedded_file_t *f = NULL;
    for (size_t i = 0; i < web_files_count; i++)
        if (strlen(web_files[i].name) == ulen && strncmp(web_files[i].name, uri, ulen) == 0)
            f = &web_files[i];
    if (!f) { httpd_resp_send_404(req); return ESP_OK; }

    if (!s_etag[0]) {
        char sha[17];
        esp_app_get_elf_sha256(sha, sizeof(sha));
        snprintf(s_etag, sizeof(s_etag), "\"%s\"", sha);
    }
    httpd_resp_set_hdr(req, "ETag", s_etag);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    char inm[32];
    if (httpd_req_get_hdr_value_str(req, "If-None-Match", inm, sizeof(inm)) == ESP_OK &&
        strcmp(inm, s_etag) == 0) {
        httpd_resp_set_status(req, "304 Not Modified");
        return httpd_resp_send(req, NULL, 0);
    }
    httpd_resp_set_type(req, f->mime);
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)f->data, f->len);
}

/* ── REST: firmware updates ───────────────────────────────────────────────── */

static esp_err_t api_ota_get(httpd_req_t *req)
{
    ota_status_t st;
    ota_update_get_status(&st);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "state",      ota_state_str(st.state));
    cJSON_AddStringToObject(o, "version",    st.version);
    cJSON_AddStringToObject(o, "build_date", st.build_date);
    cJSON_AddStringToObject(o, "latest",     st.latest);
    cJSON_AddNumberToObject(o, "progress",   st.progress);
    cJSON_AddNumberToObject(o, "last_check_s", st.last_check_s);
    cJSON_AddStringToObject(o, "message",    st.message);
    cJSON_AddStringToObject(o, "repo",       CONFIG_REFLOW_OTA_REPO);
    char *str = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    if (!str) return httpd_resp_send_500(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, str);
    free(str);
    return ESP_OK;
}

static esp_err_t api_ota_check(httpd_req_t *req)
{
    esp_err_t e = ota_update_check();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, e == ESP_OK ? "{\"ok\":true}"
                                               : "{\"ok\":false,\"message\":\"An update is in progress\"}");
}

static esp_err_t api_ota_install(httpd_req_t *req)
{
    esp_err_t e = ota_update_install();
    httpd_resp_set_type(req, "application/json");
    if (e == ESP_OK) return httpd_resp_sendstr(req, "{\"ok\":true}");
    return httpd_resp_sendstr(req, reflow_ctrl_is_active()
        ? "{\"ok\":false,\"message\":\"A reflow run is active\"}"
        : "{\"ok\":false,\"message\":\"No newer release to install\"}");
}

static esp_err_t api_ota_upload(httpd_req_t *req)
{
    return ota_update_upload(req);
}

/* ── Server start/stop ───────────────────────────────────────────────────── */

esp_err_t web_server_start(void)
{
    s_ws_mutex = xSemaphoreCreateMutex();
    settings_load();
    for (int i = 0; i < MAX_WS_CLIENTS; i++) s_ws_fds[i] = -1;
    plug_ctrl_add_result_cb(plug_result_cb);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* LWIP_MAX_SOCKETS is 16 (sdkconfig.defaults); httpd reserves 3 for itself.
       lru_purge_enable is essential, not an optimisation: without it httpd
       REFUSES new connections once every slot is held by a keep-alive socket,
       and repeated page loads will starve the WebSocket upgrade — the UI then
       sits on "connecting..." forever while plain GETs still succeed. */
    cfg.max_open_sockets  = HTTPD_MAX_OPEN_SOCKETS;
    cfg.lru_purge_enable  = true;
    cfg.max_uri_handlers  = 24;  /* default is 8; we register 19 */
    /* A firmware upload streams ~1.6 MB; the first read waits while the
       update slot is erased. */
    cfg.recv_wait_timeout = 30;
    cfg.uri_match_fn      = httpd_uri_match_wildcard;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &cfg), TAG, "httpd start");

    /* WebSocket */
    static const httpd_uri_t ws_uri = {
        .uri = "/ws", .method = HTTP_GET,
        .handler = ws_handler, .is_websocket = true,
    };
    httpd_register_uri_handler(s_server, &ws_uri);

    /* Profile REST */
    static const httpd_uri_t prof_list = {
        .uri = "/api/profiles", .method = HTTP_GET,
        .handler = api_profiles_get,
    };
    static const httpd_uri_t prof_get = {
        .uri = "/api/profiles/*", .method = HTTP_GET,
        .handler = api_profile_get,
    };
    static const httpd_uri_t prof_post = {
        .uri = "/api/profiles/*", .method = HTTP_POST,
        .handler = api_profile_post,
    };
    static const httpd_uri_t prof_del = {
        .uri = "/api/profiles/*", .method = HTTP_DELETE,
        .handler = api_profile_delete,
    };
    httpd_register_uri_handler(s_server, &prof_list);
    httpd_register_uri_handler(s_server, &prof_get);
    httpd_register_uri_handler(s_server, &prof_post);
    httpd_register_uri_handler(s_server, &prof_del);

    static const httpd_uri_t run_trace = {
        .uri = "/api/run/trace", .method = HTTP_GET,
        .handler = api_run_trace_get,
    };
    httpd_register_uri_handler(s_server, &run_trace);

    static const httpd_uri_t ota_get = {
        .uri = "/api/ota", .method = HTTP_GET, .handler = api_ota_get,
    };
    static const httpd_uri_t ota_check = {
        .uri = "/api/ota/check", .method = HTTP_POST, .handler = api_ota_check,
    };
    static const httpd_uri_t ota_install = {
        .uri = "/api/ota/install", .method = HTTP_POST, .handler = api_ota_install,
    };
    static const httpd_uri_t ota_upload = {
        .uri = "/api/ota/upload", .method = HTTP_POST, .handler = api_ota_upload,
    };
    httpd_register_uri_handler(s_server, &ota_get);
    httpd_register_uri_handler(s_server, &ota_check);
    httpd_register_uri_handler(s_server, &ota_install);
    httpd_register_uri_handler(s_server, &ota_upload);

    static const httpd_uri_t settings_get = {
        .uri = "/api/settings", .method = HTTP_GET, .handler = api_settings_get,
    };
    static const httpd_uri_t settings_post = {
        .uri = "/api/settings", .method = HTTP_POST, .handler = api_settings_post,
    };
    httpd_register_uri_handler(s_server, &settings_get);
    httpd_register_uri_handler(s_server, &settings_post);

    /* Wi-Fi REST */
    static const httpd_uri_t wifi_scan = {
        .uri = "/api/wifi/scan", .method = HTTP_GET,
        .handler = api_wifi_scan,
    };
    static const httpd_uri_t wifi_config = {
        .uri = "/api/wifi/config", .method = HTTP_GET,
        .handler = api_wifi_config_get,
    };
    static const httpd_uri_t wifi_creds = {
        .uri = "/api/wifi/credentials", .method = HTTP_POST,
        .handler = api_wifi_credentials_post,
    };
    httpd_register_uri_handler(s_server, &wifi_scan);
    httpd_register_uri_handler(s_server, &wifi_config);
    httpd_register_uri_handler(s_server, &wifi_creds);

    /* Captive portal — must come before the catch-all static handler */
    static const httpd_uri_t cp_apple = {
        .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_apple,
    };
    static const httpd_uri_t cp_204 = {
        .uri = "/generate_204", .method = HTTP_GET, .handler = captive_generate_204,
    };
    static const httpd_uri_t cp_redirect = {
        .uri = "/redirect", .method = HTTP_GET, .handler = captive_redirect,
    };
    httpd_register_uri_handler(s_server, &cp_apple);
    httpd_register_uri_handler(s_server, &cp_204);
    httpd_register_uri_handler(s_server, &cp_redirect);

    /* Static files (catch-all) */
    static const httpd_uri_t static_uri = {
        .uri = "/*", .method = HTTP_GET,
        .handler = static_file_handler,
    };
    httpd_register_uri_handler(s_server, &static_uri);

    /* Telemetry timer at 2 Hz */
    esp_timer_create_args_t timer_args = {
        .callback = telemetry_cb,
        .name     = "telemetry",
    };
    esp_timer_create(&timer_args, &s_telemetry_timer);
    esp_timer_start_periodic(s_telemetry_timer, 1000000 / TELEMETRY_HZ);

    ESP_LOGI(TAG, "Web server started");
    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (s_telemetry_timer) esp_timer_stop(s_telemetry_timer);
    if (s_server) httpd_stop(s_server);
    s_server = NULL;
    return ESP_OK;
}
