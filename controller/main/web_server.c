#include "web_server.h"
#include "reflow_ctrl.h"
#include "reflow_profile.h"
#include "zigbee_plug.h"
#include "wifi_manager.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include "esp_spiffs.h"
#include <string.h>
#include <stdio.h>

#define MAX_WS_CLIENTS   4
#define MAX_RESP_BUF     4096
#define TELEMETRY_HZ     2
#define PAIR_DURATION_S  60

static const char *TAG = "web_server";

static httpd_handle_t   s_server = NULL;
static int              s_ws_fds[MAX_WS_CLIENTS];
static SemaphoreHandle_t s_ws_mutex;
static esp_timer_handle_t s_telemetry_timer;

/* ── WebSocket client list ───────────────────────────────────────────────── */

static void ws_client_add(int fd)
{
    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == -1) { s_ws_fds[i] = fd; break; }
    }
    xSemaphoreGive(s_ws_mutex);
}

static void ws_client_remove(int fd)
{
    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == fd) { s_ws_fds[i] = -1; break; }
    }
    xSemaphoreGive(s_ws_mutex);
}

void web_server_broadcast(const char *json)
{
    if (!s_server) return;

    httpd_ws_frame_t frame = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json,
        .len     = strlen(json),
    };

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == -1) continue;
        esp_err_t err = httpd_ws_send_frame_async(s_server, s_ws_fds[i], &frame);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "WS client %d send failed (%s), closing", s_ws_fds[i], esp_err_to_name(err));
            httpd_sess_trigger_close(s_server, s_ws_fds[i]);
            s_ws_fds[i] = -1;
        }
    }
    xSemaphoreGive(s_ws_mutex);
}

/* ── Telemetry ───────────────────────────────────────────────────────────── */

static const char *state_str(ctrl_state_t s)
{
    switch (s) {
        case CTRL_STATE_RUNNING:  return "running";
        case CTRL_STATE_COMPLETE: return "complete";
        case CTRL_STATE_ERROR:    return "error";
        default:                  return "idle";
    }
}

static void telemetry_cb(void *arg)
{
    ctrl_status_t st;
    reflow_ctrl_get_status(&st);

    char ip[20] = "0.0.0.0";
    wifi_manager_get_ip(ip, sizeof(ip));

    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "type",        "telemetry");
    cJSON_AddNumberToObject(obj, "temp",        (double)st.temp);
    cJSON_AddNumberToObject(obj, "setpoint",    (double)st.setpoint);
    cJSON_AddNumberToObject(obj, "elapsed",     st.elapsed_s);
    cJSON_AddStringToObject(obj, "phase",       st.phase);
    cJSON_AddStringToObject(obj, "state",       state_str(st.state));
    cJSON_AddBoolToObject  (obj, "sensor_ok",   st.sensor_ok);
    cJSON_AddBoolToObject  (obj, "plug_paired", zigbee_plug_is_paired());
    cJSON_AddBoolToObject  (obj, "plug_on",     st.plug_on);
    cJSON_AddNumberToObject(obj, "duty_steps",  st.duty_steps);
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
        if (cJSON_IsString(pname)) {
            reflow_profile_t profile;
            if (profile_get_by_name(pname->valuestring, &profile) == ESP_OK) {
                reflow_ctrl_start(&profile);
            }
        }
    } else if (strcmp(t, "stop") == 0) {
        reflow_ctrl_stop();
    } else if (strcmp(t, "start_pairing") == 0) {
        zigbee_plug_start_pairing(PAIR_DURATION_S, pairing_result_cb, pairing_countdown_cb);
    } else if (strcmp(t, "unpair") == 0) {
        zigbee_plug_unpair();
    } else if (strcmp(t, "plug_set") == 0) {
        cJSON *on_item = cJSON_GetObjectItem(msg, "on");
        if (cJSON_IsBool(on_item)) {
            reflow_ctrl_set_plug(cJSON_IsTrue(on_item));
        }
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

static esp_err_t api_wifi_credentials_post(httpd_req_t *req)
{
    char buf[256] = {0};
    int r = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (r <= 0) return ESP_FAIL;
    buf[r] = '\0';

    cJSON *obj  = cJSON_Parse(buf);
    cJSON *ssid = cJSON_GetObjectItem(obj, "ssid");
    cJSON *pass = cJSON_GetObjectItem(obj, "password");

    if (!cJSON_IsString(ssid)) {
        cJSON_Delete(obj);
        httpd_resp_sendstr(req, "{\"ok\":false}");
        return ESP_OK;
    }

    wifi_manager_set_credentials(ssid->valuestring,
                                  cJSON_IsString(pass) ? pass->valuestring : "");
    cJSON_Delete(obj);
    httpd_resp_set_type(req, "application/json");
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

/* ── Static file server ───────────────────────────────────────────────────── */

static const char *mime_type(const char *path)
{
    const char *ext = strrchr(path, '.');
    if (!ext) return "application/octet-stream";
    if (strcmp(ext, ".html") == 0) return "text/html";
    if (strcmp(ext, ".css")  == 0) return "text/css";
    if (strcmp(ext, ".js")   == 0) return "application/javascript";
    if (strcmp(ext, ".json") == 0) return "application/json";
    return "application/octet-stream";
}

static esp_err_t static_file_handler(httpd_req_t *req)
{
    char path[640];
    const char *uri = req->uri;
    if (strcmp(uri, "/") == 0) uri = "/index.html";
    ESP_LOGI(TAG, "GET %s", uri);
    snprintf(path, sizeof(path), "/spiffs/www%.512s", uri);

    FILE *f = fopen(path, "r");
    if (!f) { httpd_resp_send_404(req); return ESP_OK; }

    httpd_resp_set_type(req, mime_type(path));

    char *chunk = malloc(512);
    if (!chunk) { fclose(f); return httpd_resp_send_500(req); }

    size_t n;
    while ((n = fread(chunk, 1, 512, f)) > 0) {
        httpd_resp_send_chunk(req, chunk, n);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    free(chunk);
    fclose(f);
    return ESP_OK;
}

/* ── Server start/stop ───────────────────────────────────────────────────── */

esp_err_t web_server_start(void)
{
    s_ws_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < MAX_WS_CLIENTS; i++) s_ws_fds[i] = -1;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets  = 7;   /* LWIP_MAX_SOCKETS(10) - 3 reserved by httpd */
    cfg.max_uri_handlers  = 14;  /* default is 8, we register 11 handlers */
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

    /* Wi-Fi REST */
    static const httpd_uri_t wifi_scan = {
        .uri = "/api/wifi/scan", .method = HTTP_GET,
        .handler = api_wifi_scan,
    };
    static const httpd_uri_t wifi_creds = {
        .uri = "/api/wifi/credentials", .method = HTTP_POST,
        .handler = api_wifi_credentials_post,
    };
    httpd_register_uri_handler(s_server, &wifi_scan);
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
