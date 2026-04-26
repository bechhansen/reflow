#include "zigbee_plug.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "sdkconfig.h"
#include <string.h>

/* These helper macros follow the pattern in the IDF Zigbee examples */
#define MAX_CHILDREN                10
#define INSTALLCODE_POLICY_ENABLE   false
#define ESP_ZB_PRIMARY_CHANNEL_MASK (1l << 26)  /* ch 26 = 2480 MHz, no Wi-Fi overlap */

#define ESP_ZB_ZC_CONFIG() {                                    \
    .esp_zb_role         = ESP_ZB_DEVICE_TYPE_COORDINATOR,      \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
    .nwk_cfg.zczr_cfg    = { .max_children = MAX_CHILDREN },    \
}

#define ESP_ZB_DEFAULT_RADIO_CONFIG() {                         \
    .radio_mode = ZB_RADIO_MODE_NATIVE,                         \
}

#define ESP_ZB_DEFAULT_HOST_CONFIG() {                          \
    .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,       \
}

#define NVS_NS           "zigbee_plug"
#define NVS_KEY_ADDR     "short_addr"
#define NVS_KEY_EP       "endpoint"
#define PAIR_DURATION_S  60
#define LOCAL_ENDPOINT   CONFIG_REFLOW_ZB_ENDPOINT

static const char *TAG = "zigbee_plug";

static uint16_t s_plug_addr   = 0xFFFF;
static uint8_t  s_plug_ep     = 0xFF;
static bool     s_pairing     = false;
static zigbee_pairing_cb_t   s_pair_cb      = NULL;
static zigbee_countdown_cb_t s_countdown_cb = NULL;

/* ── NVS helpers ─────────────────────────────────────────────────────────── */

static void load_pairing_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) != ESP_OK) return;
    nvs_get_u16(nvs, NVS_KEY_ADDR, &s_plug_addr);
    uint8_t ep = 0;
    nvs_get_u8(nvs, NVS_KEY_EP, &ep);
    s_plug_ep = ep;
    nvs_close(nvs);
    if (s_plug_addr != 0xFFFF)
        ESP_LOGI(TAG, "Restored pairing: addr=0x%04x ep=%d", s_plug_addr, s_plug_ep);
}

static void save_pairing_to_nvs(uint16_t addr, uint8_t ep)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u16(nvs, NVS_KEY_ADDR, addr);
    nvs_set_u8(nvs, NVS_KEY_EP, ep);
    nvs_commit(nvs);
    nvs_close(nvs);
}

/* ── BDB top-level commissioning callback ────────────────────────────────── */

static void bdb_commissioning_cb(uint8_t mode_mask)
{
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

/* ── Device announce / match callback ───────────────────────────────────── */

static void plug_find_cb(esp_zb_zdp_status_t status, uint16_t addr, uint8_t ep, void *ctx)
{
    if (status != ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Did not find On/Off cluster on joined device");
        return;
    }
    ESP_LOGI(TAG, "Paired plug: addr=0x%04x ep=%d", addr, ep);
    s_plug_addr    = addr;
    s_plug_ep      = ep;
    s_pairing      = false;
    s_countdown_cb = NULL;
    save_pairing_to_nvs(addr, ep);

    if (s_pair_cb) {
        s_pair_cb(true, addr, ep);
        s_pair_cb = NULL;
    }
}

/* ── App signal handler (called from Zigbee task) ────────────────────────── */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p = signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig = (esp_zb_app_signal_type_t)*p;

    switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "Zigbee coordinator started (%s)",
                     esp_zb_bdb_is_factory_new() ? "factory-new" : "rebooted");
            if (esp_zb_bdb_is_factory_new()) {
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                esp_zb_bdb_open_network(PAIR_DURATION_S);
            }
        } else {
            ESP_LOGE(TAG, "Zigbee stack error: %s", esp_err_to_name(status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "Network formed (PAN 0x%04x, ch %d)",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Formation failed, retrying...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (status == ESP_OK)
            ESP_LOGI(TAG, "Network steering active");
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *dev =
            (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p);
        ESP_LOGI(TAG, "Device joined: 0x%04x", dev->device_short_addr);
        if (s_pairing) {
            esp_zb_zdo_match_desc_req_param_t req = {
                .dst_nwk_addr    = dev->device_short_addr,
                .addr_of_interest = dev->device_short_addr,
            };
            esp_zb_zdo_find_on_off_light(&req, plug_find_cb, NULL);
        }
        break;
    }

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t remaining = *(uint8_t *)esp_zb_app_signal_get_params(p);
        if (s_pairing) {
            if (s_countdown_cb) s_countdown_cb(remaining);
            if (remaining == 0) {
                ESP_LOGW(TAG, "Pairing window closed without a device joining");
                s_pairing = false;
                if (s_pair_cb) { s_pair_cb(false, 0, 0); s_pair_cb = NULL; }
                s_countdown_cb = NULL;
            }
        }
        break;
    }

    default:
        ESP_LOGD(TAG, "Signal: %s (0x%x)", esp_zb_zdo_signal_to_string(sig), sig);
        break;
    }
}

/* ── Zigbee task ─────────────────────────────────────────────────────────── */

static void zigbee_task(void *arg)
{
    esp_zb_cfg_t cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&cfg);

    esp_zb_on_off_switch_cfg_t sw_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_ep_list_t *ep_list = esp_zb_on_off_switch_ep_create(LOCAL_ENDPOINT, &sw_cfg);
    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    load_pairing_from_nvs();

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t zigbee_plug_init(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_RETURN_ON_ERROR(esp_zb_platform_config(&config), TAG, "platform config");
    xTaskCreate(zigbee_task, "zigbee", 4096, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t zigbee_plug_set(bool on)
{
    if (s_plug_addr == 0xFFFF) return ESP_ERR_INVALID_STATE;

    esp_zb_zcl_on_off_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = s_plug_addr,
            .dst_endpoint          = s_plug_ep,
            .src_endpoint          = LOCAL_ENDPOINT,
        },
        .address_mode  = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_on_off_cmd_req(&cmd);
    esp_zb_lock_release();
    return err;
}

bool zigbee_plug_is_paired(void)
{
    return s_plug_addr != 0xFFFF;
}

esp_err_t zigbee_plug_start_pairing(uint8_t duration_s, zigbee_pairing_cb_t cb, zigbee_countdown_cb_t countdown_cb)
{
    s_pair_cb      = cb;
    s_countdown_cb = countdown_cb;
    s_pairing      = true;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_bdb_open_network(duration_s);
    esp_zb_lock_release();
    ESP_LOGI(TAG, "Pairing window open for %d seconds", duration_s);
    return ESP_OK;
}

esp_err_t zigbee_plug_unpair(void)
{
    s_plug_addr = 0xFFFF;
    s_plug_ep   = 0xFF;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, NVS_KEY_ADDR);
        nvs_erase_key(nvs, NVS_KEY_EP);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Plug unpaired");
    return ESP_OK;
}
