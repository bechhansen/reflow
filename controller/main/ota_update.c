#include "ota_update.h"
#include "reflow_ctrl.h"
#include "plug_ctrl.h"
#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Firmware updates over Wi-Fi ───────────────────────────────────
   See ota_update.h. One task does the network work (checks and GitHub
   installs); uploads run in the HTTP handler. Status is a snapshot under a
   spinlock, read by the REST API and telemetry. */

static const char *TAG = "ota";

#define ASSET_NAME        "reflow-controller.bin"
#define FIRST_CHECK_MS    (60 * 1000)
#define CHECK_PERIOD_MS   (24 * 3600 * 1000)
#define CONFIRM_AFTER_US  (30 * 1000000LL)
#define MAX_JSON          (32 * 1024)
#define HEATER_OFF_WAIT_S 15   /* longest wait for the plug to confirm OFF */
#define UNKNOWN_WAIT_S    5    /* ... when it does not answer at all */

static portMUX_TYPE      s_mux = portMUX_INITIALIZER_UNLOCKED;
static ota_status_t      s_st;
static int64_t           s_last_check_us = -1;
static char              s_asset_url[512];
static SemaphoreHandle_t s_wake;           /* check or install requested */
static volatile bool     s_install_req;

/* ── Status ────────────────────────────────────────────────────────── */

static void set_state(ota_state_t st, const char *msg)
{
    taskENTER_CRITICAL(&s_mux);
    s_st.state = st;
    if (msg) strlcpy(s_st.message, msg, sizeof(s_st.message));
    if (st != OTA_INSTALLING) s_st.progress = 0;
    taskEXIT_CRITICAL(&s_mux);
}

static void set_progress(int pct)
{
    taskENTER_CRITICAL(&s_mux);
    s_st.progress = pct;
    taskEXIT_CRITICAL(&s_mux);
}

void ota_update_get_status(ota_status_t *out)
{
    taskENTER_CRITICAL(&s_mux);
    *out = s_st;
    int64_t last = s_last_check_us;
    taskEXIT_CRITICAL(&s_mux);
    out->last_check_s = last < 0 ? -1 : (int)((esp_timer_get_time() - last) / 1000000);
}

bool ota_update_busy(void)
{
    taskENTER_CRITICAL(&s_mux);
    bool b = s_st.state == OTA_INSTALLING || s_st.state == OTA_REBOOTING;
    taskEXIT_CRITICAL(&s_mux);
    return b;
}

const char *ota_state_str(ota_state_t s)
{
    switch (s) {
        case OTA_CHECKING:   return "checking";
        case OTA_UP_TO_DATE: return "up_to_date";
        case OTA_AVAILABLE:  return "available";
        case OTA_INSTALLING: return "installing";
        case OTA_REBOOTING:  return "rebooting";
        case OTA_ERROR:      return "error";
        default:             return "idle";
    }
}

/* ── Versions ──────────────────────────────────────────────────────── */

/* "v1.2.3", "1.2.3", "v1.2.3-4-gabc" -> 1,2,3. Anything else (a bare commit
   hash from an untagged build) counts as 0.0.0, so any release is newer. */
static void parse_version(const char *s, int v[3])
{
    v[0] = v[1] = v[2] = 0;
    if (*s == 'v' || *s == 'V') s++;
    if (sscanf(s, "%d.%d.%d", &v[0], &v[1], &v[2]) != 3) v[0] = v[1] = v[2] = 0;
}

/* Production release: exactly vX.Y.Z (or X.Y.Z). Alpha, beta, rc and other
   suffixed tags are never downloaded; they can only be installed by upload. */
static bool is_production(const char *tag)
{
    int a, b, c, n = 0;
    if (*tag == 'v' || *tag == 'V') tag++;
    return sscanf(tag, "%d.%d.%d%n", &a, &b, &c, &n) == 3 && tag[n] == '\0';
}

/* A pre-release of its X.Y.Z: "v1.2.3-beta2", "v1.2.3-rc1-4-gabc". A local
   build past a release ("v1.2.3-4-gabc") is not one. */
static bool is_prerelease(const char *s)
{
    int a, b, c, n = 0;
    if (*s == 'v' || *s == 'V') s++;
    if (sscanf(s, "%d.%d.%d%n", &a, &b, &c, &n) != 3 || s[n] != '-') return false;
    s += n + 1;
    return !strncmp(s, "alpha", 5) || !strncmp(s, "beta", 4) || !strncmp(s, "rc", 2);
}

/* Newer by X.Y.Z; at equal numbers a production release is newer than a
   pre-release of it (v1.2.3 > v1.2.3-beta2), so beta testers are offered the
   final release. A local build counts as its X.Y.Z. */
static bool is_newer(const char *candidate, const char *running)
{
    int a[3], b[3];
    parse_version(candidate, a);
    parse_version(running, b);
    for (int i = 0; i < 3; i++)
        if (a[i] != b[i]) return a[i] > b[i];
    return is_prerelease(running) && !is_prerelease(candidate);
}

/* ── Safety before writing ─────────────────────────────────────────── */

static bool confirm_running(const char *why);

/* Heater off, and confirmed if the plug answers. Only a run blocks an
   update: with no run active, an unreachable plug (paired but unplugged or out
   of range) is no reason to refuse. The update cannot make its state worse, and
   after the reboot the controller keeps sending OFF until the plug confirms. */
static esp_err_t prepare_install(void)
{
    if (reflow_ctrl_is_active()) return ESP_ERR_INVALID_STATE;
    /* A firmware not yet confirmed cannot start another update (ESP-IDF
       refuses while rollback is pending). Being asked for one from the web UI
       shows it booted, joined the network and serves pages: confirm it now. */
    if (!confirm_running("an update was requested from the web UI")) {
        set_state(OTA_ERROR, "This firmware is a test build that never confirms itself; reset to roll back");
        return ESP_ERR_INVALID_STATE;
    }
    set_state(OTA_INSTALLING, "Switching the heater off");
    reflow_ctrl_stop();                       /* force-off until confirmed */
    int unknown = 0;
    for (int i = 0; i < HEATER_OFF_WAIT_S * 4; i++) {
        plug_status_t ps;
        plug_ctrl_get_status(&ps);
        if (!ps.available) return ESP_OK;     /* no plug: nothing to switch */
        if (ps.state == PLUG_OFF && !ps.pending) return ESP_OK;
        /* Not answering at all: give it a few polls, then go on. */
        unknown = (ps.state == PLUG_UNKNOWN && !ps.pending) ? unknown + 1 : 0;
        if (unknown >= UNKNOWN_WAIT_S * 4) break;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    ESP_LOGW(TAG, "heater not confirmed off (plug not answering); no run active, updating anyway");
    return ESP_OK;
}

static void restart_soon(void *arg) { esp_restart(); }

static void reboot_after(int ms)
{
    set_state(OTA_REBOOTING, "Restarting into the new firmware");
    const esp_timer_create_args_t a = { .callback = restart_soon, .name = "ota_reboot" };
    esp_timer_handle_t t;
    if (esp_timer_create(&a, &t) == ESP_OK) esp_timer_start_once(t, (uint64_t)ms * 1000);
    else esp_restart();
}

/* ── GitHub ────────────────────────────────────────────────────────── */

static void check_url(char *buf, size_t len)
{
    if (CONFIG_REFLOW_OTA_CHECK_URL[0])
        strlcpy(buf, CONFIG_REFLOW_OTA_CHECK_URL, len);
    else
        snprintf(buf, len, "https://api.github.com/repos/%s/releases/latest", CONFIG_REFLOW_OTA_REPO);
}

static void do_check(void)
{
    set_state(OTA_CHECKING, "");
    char url[160];
    check_url(url, sizeof(url));

    esp_http_client_config_t cfg = {
        .url               = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 15000,
        .buffer_size       = 4096,
        .buffer_size_tx    = 1024,
        .user_agent        = "reflow-controller",
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    char *body = NULL;
    const char *err = NULL;
    if (!c) { err = "HTTP client init failed"; goto done; }
    esp_http_client_set_header(c, "Accept", "application/vnd.github+json");
    if (esp_http_client_open(c, 0) != ESP_OK) { err = "Could not reach the release server"; goto done; }
    esp_http_client_fetch_headers(c);
    int code = esp_http_client_get_status_code(c);
    if (code == 404) { err = "No release found (repository private or no production release yet)"; goto done; }
    if (code != 200) { err = "Release server answered an error"; goto done; }

    body = malloc(MAX_JSON + 1);
    if (!body) { err = "Out of memory"; goto done; }
    int n = 0, r;
    while (n < MAX_JSON && (r = esp_http_client_read(c, body + n, MAX_JSON - n)) > 0) n += r;
    body[n] = '\0';

    cJSON *j = cJSON_Parse(body);
    cJSON *tag = j ? cJSON_GetObjectItem(j, "tag_name") : NULL;
    cJSON *assets = j ? cJSON_GetObjectItem(j, "assets") : NULL;
    const char *asset_url = NULL;
    cJSON *a;
    cJSON_ArrayForEach(a, assets) {
        cJSON *nm = cJSON_GetObjectItem(a, "name");
        cJSON *u  = cJSON_GetObjectItem(a, "browser_download_url");
        if (cJSON_IsString(nm) && cJSON_IsString(u) && strcmp(nm->valuestring, ASSET_NAME) == 0)
            asset_url = u->valuestring;
    }
    if (!cJSON_IsString(tag)) {
        err = "Unexpected reply from the release server";
    } else {
        bool newer = asset_url && is_production(tag->valuestring) &&
                     is_newer(tag->valuestring, s_st.version);
        taskENTER_CRITICAL(&s_mux);
        strlcpy(s_st.latest, tag->valuestring, sizeof(s_st.latest));
        taskEXIT_CRITICAL(&s_mux);
        if (newer) strlcpy(s_asset_url, asset_url, sizeof(s_asset_url));
        ESP_LOGI(TAG, "latest release %s (running %s)%s", tag->valuestring, s_st.version,
                 newer ? ": update available" : "");
        set_state(newer ? OTA_AVAILABLE : OTA_UP_TO_DATE, "");
        if (!asset_url) set_state(OTA_UP_TO_DATE, "The latest release has no " ASSET_NAME);
        else if (!is_production(tag->valuestring))
            set_state(OTA_UP_TO_DATE, "The latest release is not a production release; upload it manually");
    }
    cJSON_Delete(j);

done:
    free(body);
    if (c) { esp_http_client_close(c); esp_http_client_cleanup(c); }
    taskENTER_CRITICAL(&s_mux);
    s_last_check_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_mux);
    if (err) {
        ESP_LOGW(TAG, "check failed: %s", err);
        set_state(OTA_ERROR, err);
    }
}

static void do_install(void)
{
    if (prepare_install() != ESP_OK) {
        if (!ota_update_busy() && s_st.state != OTA_ERROR)
            set_state(OTA_ERROR, "A reflow run is active; update not started");
        return;
    }
    set_state(OTA_INSTALLING, "Downloading");
    ESP_LOGI(TAG, "installing %s from %s", s_st.latest, s_asset_url);

    esp_http_client_config_t http = {
        .url               = s_asset_url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 30000,
        .buffer_size       = 4096,
        .buffer_size_tx    = 4096,     /* GitHub redirects to a long signed URL */
        .user_agent        = "reflow-controller",
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t cfg = { .http_config = &http };
    esp_https_ota_handle_t h = NULL;
    esp_err_t err = esp_https_ota_begin(&cfg, &h);
    if (err == ESP_OK) {
        int size = esp_https_ota_get_image_size(h);
        while ((err = esp_https_ota_perform(h)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            int got = esp_https_ota_get_image_len_read(h);
            if (size > 0) set_progress(got * 100 / size);
        }
        if (err == ESP_OK && !esp_https_ota_is_complete_data_received(h)) err = ESP_FAIL;
        if (err == ESP_OK) err = esp_https_ota_finish(h);     /* verifies, sets boot slot */
        else esp_https_ota_abort(h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "install failed: %s", esp_err_to_name(err));
        char msg[96];
        snprintf(msg, sizeof(msg), "Update failed (%s); still running %s", esp_err_to_name(err), s_st.version);
        set_state(OTA_ERROR, msg);
        return;
    }
    ESP_LOGI(TAG, "installed; rebooting");
    reboot_after(1500);
}

static void ota_task(void *arg)
{
    TickType_t wait = pdMS_TO_TICKS(FIRST_CHECK_MS);
    for (;;) {
        xSemaphoreTake(s_wake, wait);
        wait = pdMS_TO_TICKS(CHECK_PERIOD_MS);
        if (ota_update_busy()) continue;
        if (s_install_req) {
            s_install_req = false;
            do_install();
        } else if (wifi_manager_get_mode() == WIFI_MGR_STA) {
            do_check();
        }
    }
}

esp_err_t ota_update_check(void)
{
    if (ota_update_busy()) return ESP_ERR_INVALID_STATE;
    xSemaphoreGive(s_wake);
    return ESP_OK;
}

esp_err_t ota_update_install(void)
{
    if (ota_update_busy() || s_st.state != OTA_AVAILABLE || reflow_ctrl_is_active())
        return ESP_ERR_INVALID_STATE;
    s_install_req = true;
    xSemaphoreGive(s_wake);
    return ESP_OK;
}

/* ── Upload ────────────────────────────────────────────────────────── */

static esp_err_t upload_reply(httpd_req_t *req, bool ok, const char *msg)
{
    char buf[160];
    snprintf(buf, sizeof(buf), "{\"ok\":%s,\"message\":\"%s\"}", ok ? "true" : "false", msg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

esp_err_t ota_update_upload(httpd_req_t *req)
{
    if (ota_update_busy())          return upload_reply(req, false, "An update is already in progress");
    if (reflow_ctrl_is_active())    return upload_reply(req, false, "A reflow run is active");
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part)                      return upload_reply(req, false, "No update slot (partition table without OTA)");
    if (req->content_len == 0 || req->content_len > part->size)
                                    return upload_reply(req, false, "File is empty or larger than the update slot");
    if (prepare_install() != ESP_OK) return upload_reply(req, false, "A reflow run is active, or this test build cannot update");

    set_state(OTA_INSTALLING, "Writing uploaded firmware");
    esp_ota_handle_t h;
    esp_err_t err = esp_ota_begin(part, req->content_len, &h);
    char *buf = err == ESP_OK ? malloc(4096) : NULL;
    size_t left = req->content_len;
    if (err == ESP_OK && !buf) err = ESP_ERR_NO_MEM;
    while (err == ESP_OK && left > 0) {
        int r = httpd_req_recv(req, buf, left < 4096 ? left : 4096);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) { err = ESP_FAIL; break; }
        err = esp_ota_write(h, buf, r);
        left -= r;
        set_progress((int)((req->content_len - left) * 100 / req->content_len));
    }
    free(buf);
    if (err == ESP_OK) err = esp_ota_end(h);      /* verifies the image */
    else esp_ota_abort(h);

    /* Only this project's firmware: an image for another device would boot,
       fail to confirm and roll back, but better not to boot it at all. */
    esp_app_desc_t desc;
    if (err == ESP_OK && esp_ota_get_partition_description(part, &desc) == ESP_OK &&
        strcmp(desc.project_name, esp_app_get_description()->project_name) != 0) {
        set_state(OTA_ERROR, "That file is not firmware for this controller");
        return upload_reply(req, false, "That file is not firmware for this controller");
    }
    if (err == ESP_OK) err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        char msg[96];
        snprintf(msg, sizeof(msg), "Upload failed (%s); still running %s", esp_err_to_name(err), s_st.version);
        set_state(OTA_ERROR, msg);
        return upload_reply(req, false, msg);
    }
    ESP_LOGI(TAG, "uploaded %s; rebooting", desc.version);
    upload_reply(req, true, "Installed; restarting");
    reboot_after(1500);
    return ESP_OK;
}

/* ── Confirmation of a new firmware ────────────────────────────────── */

/* Mark a freshly installed firmware as good. Returns false only for the
   test build that must never confirm (it stays pending, so the next reset
   rolls back). */
static bool confirm_running(const char *why)
{
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) != ESP_OK ||
        st != ESP_OTA_IMG_PENDING_VERIFY)
        return true;                          /* nothing to confirm */
#if CONFIG_REFLOW_OTA_TEST_NO_CONFIRM
    ESP_LOGW(TAG, "TEST build: not confirming (%s); the next reset rolls back", why);
    return false;
#else
    esp_ota_mark_app_valid_cancel_rollback();
    ESP_LOGI(TAG, "new firmware %s confirmed (%s)", s_st.version, why);
    return true;
#endif
}

static void confirm_cb(void *arg) { confirm_running("ran 30 s"); }

esp_err_t ota_update_init(void)
{
    const esp_app_desc_t *d = esp_app_get_description();
    strlcpy(s_st.version, d->version, sizeof(s_st.version));
    snprintf(s_st.build_date, sizeof(s_st.build_date), "%s %s", d->date, d->time);
    s_st.state = OTA_IDLE;

    /* A firmware that failed to confirm itself was rolled back. */
    const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
    esp_app_desc_t bd;
    if (bad && esp_ota_get_partition_description(bad, &bd) == ESP_OK)
        snprintf(s_st.message, sizeof(s_st.message),
                 "Firmware %s did not start properly and was rolled back", bd.version);

    /* A new firmware counts as good once it has served the web UI for 30 s;
       Wi-Fi is not required, so a router that is off does not undo a good
       update. */
    const esp_timer_create_args_t a = { .callback = confirm_cb, .name = "ota_confirm" };
    esp_timer_handle_t t;
    if (esp_timer_create(&a, &t) == ESP_OK) esp_timer_start_once(t, CONFIRM_AFTER_US);

    s_wake = xSemaphoreCreateBinary();
    xTaskCreate(ota_task, "ota", 8192, NULL, 4, NULL);
    ESP_LOGI(TAG, "running %s (built %s)", s_st.version, s_st.build_date);
    return ESP_OK;
}
