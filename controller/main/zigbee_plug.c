#include "zigbee_plug.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "aps/esp_zigbee_aps.h"
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
#define NVS_KEY_IEEE     "ieee_addr"
#define PAIR_DURATION_S  60
#define LOCAL_ENDPOINT   CONFIG_REFLOW_ZB_ENDPOINT

static const char *TAG = "zigbee_plug";

static uint16_t s_plug_addr   = 0xFFFF;
static uint8_t  s_plug_ep     = 0xFF;
/* The 64-bit IEEE address is the plug's permanent identity. The 16-bit short
   address is assigned by the network and CHANGES when the device rejoins — so
   a pairing stored only as a short address is silently orphaned by a power cut,
   and every command after that is addressed to a device that no longer exists.
   That happened here: a whole run's worth of OFF commands went nowhere while
   an iron sat powered, because the API returns a sequence number rather than a
   delivery result. Commands are now addressed by IEEE. */
static esp_zb_ieee_addr_t s_plug_ieee = {0};
static bool     s_have_ieee   = false;
static esp_zb_ieee_addr_t s_annce_ieee = {0};   /* seen in the join signal */
static bool     s_annce_valid = false;
static bool     s_pairing     = false;
static zigbee_pairing_cb_t   s_pair_cb      = NULL;
static zigbee_countdown_cb_t s_countdown_cb = NULL;
static const zigbee_plug_listener_t *s_listener = NULL;
static bool     s_started     = false;   /* zigbee_plug_init() ran: the stack lock exists */

/* ── NVS helpers ─────────────────────────────────────────────────────────── */

static void load_pairing_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) != ESP_OK) return;
    nvs_get_u16(nvs, NVS_KEY_ADDR, &s_plug_addr);
    uint8_t ep = 0;
    nvs_get_u8(nvs, NVS_KEY_EP, &ep);
    s_plug_ep = ep;
    size_t ilen = sizeof(s_plug_ieee);
    if (nvs_get_blob(nvs, NVS_KEY_IEEE, s_plug_ieee, &ilen) == ESP_OK && ilen == sizeof(s_plug_ieee))
        s_have_ieee = true;
    nvs_close(nvs);
    if (s_plug_addr != 0xFFFF)
        ESP_LOGI(TAG, "Restored pairing: addr=0x%04x ep=%d ieee=%s",
                 s_plug_addr, s_plug_ep, s_have_ieee ? "yes" : "NO (short address only)");
}

static void save_pairing_to_nvs(uint16_t addr, uint8_t ep)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u16(nvs, NVS_KEY_ADDR, addr);
    nvs_set_u8(nvs, NVS_KEY_EP, ep);
    if (s_have_ieee) nvs_set_blob(nvs, NVS_KEY_IEEE, s_plug_ieee, sizeof(s_plug_ieee));
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
    if (s_annce_valid) {
        memcpy(s_plug_ieee, s_annce_ieee, sizeof(s_plug_ieee));
        s_have_ieee = true;
    }
    ESP_LOGI(TAG, "Paired plug: addr=0x%04x ep=%d ieee=%s", addr, ep,
             s_have_ieee ? "captured" : "NOT captured");
    s_plug_addr    = addr;
    s_plug_ep      = ep;
    s_pairing      = false;
    s_countdown_cb = NULL;
    save_pairing_to_nvs(addr, ep);
    if (s_listener && s_listener->on_paired) s_listener->on_paired();

    if (s_pair_cb) {
        s_pair_cb(true, addr, ep);
        s_pair_cb = NULL;
    }
}

/* ── App signal handler (called from Zigbee task) ────────────────────────── */

/* Defined below, with the commands they relate to. */
static void network_up_cb(uint8_t param);
static esp_err_t zcl_action_handler(esp_zb_core_action_callback_id_t id, const void *message);

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
            /* Tell the plug service the network is up; it switches a paired
               plug off and confirms it. 3 s lets the network settle first. */
            esp_zb_scheduler_alarm(network_up_cb, 0, 3000);
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
        memcpy(s_annce_ieee, dev->ieee_addr, sizeof(s_annce_ieee));
        s_annce_valid = true;
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

/* ── Radio coexistence: why the coex priorities are left at driver defaults ──

   The C6 has one 2.4 GHz radio and Wi-Fi owns it most of the time: the plug's
   replies are reliably received only in the ~15 ms after WE transmit (see
   plug_fsm.h for how the read cadence exploits that). Measured on this board,
   with esp_ieee802154_set_coex_config():
     idle=MIDDLE                one fast run, not reproducible; worse with fast reads
     txrx=MIDDLE                no Zigbee gain, Wi-Fi telemetry broke up
     idle/txrx/txrx_at=HIGH     Wi-Fi dead (WebSocket dropped)
   Wi-Fi modem sleep (WIFI_PS_MIN_MODEM) made no difference either. */

/* ── Zigbee task ─────────────────────────────────────────────────────────── */

static void zigbee_task(void *arg)
{
    esp_zb_cfg_t cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&cfg);

    esp_zb_on_off_switch_cfg_t sw_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_ep_list_t *ep_list = esp_zb_on_off_switch_ep_create(LOCAL_ENDPOINT, &sw_cfg);
    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_core_action_handler_register(zcl_action_handler);

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
    s_started = true;
    return ESP_OK;
}

void zigbee_plug_set_listener(const zigbee_plug_listener_t *listener)
{
    s_listener = listener;
}

/* ── Plug commands (Zigbee context only) ─────────────────────────────────── */

/* Address the plug by IEEE where known: a plug that rejoined has a different
   short address, and a command sent to the old one silently goes nowhere.
   Also refreshes the cached short address, which response filtering and
   diagnostics use. */
static void fill_plug_dst(esp_zb_zcl_basic_cmd_t *basic, esp_zb_zcl_address_mode_t *mode)
{
    basic->dst_endpoint = s_plug_ep;
    basic->src_endpoint = LOCAL_ENDPOINT;
    if (s_have_ieee) {
        memcpy(basic->dst_addr_u.addr_long, s_plug_ieee, sizeof(basic->dst_addr_u.addr_long));
        *mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;

        uint16_t nwk = esp_zb_address_short_by_ieee(s_plug_ieee);
        if (nwk == 0xFFFF) {
            ESP_LOGW(TAG, "plug not resolvable on the network right now "
                          "(stored short addr 0x%04x) — frame may not arrive", s_plug_addr);
        } else if (nwk != s_plug_addr) {
            ESP_LOGW(TAG, "plug short address changed 0x%04x -> 0x%04x (rejoined)",
                     s_plug_addr, nwk);
            s_plug_addr = nwk;
            save_pairing_to_nvs(nwk, s_plug_ep);
        }
    } else {
        basic->dst_addr_u.addr_short = s_plug_addr;
        *mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    }
}

/* On/Off goes out as a raw APS frame WITHOUT APS acknowledgement.
   esp_zb_zcl_on_off_cmd_req() requests an APS ACK, and under Wi-Fi
   coexistence the plug's ACK is often lost, so the stack retransmits the
   command 1-3 s later. Measured on hardware: an OFF sent ~2.5 s after an ON
   was confirmed, then the retransmitted ON arrived and switched the relay
   back on (14 s of full-power heating under a confirmed OFF). Without the
   APS ACK there is nothing to retransmit late; the MAC layer still retries
   within milliseconds. A command that is lost outright is caught by the
   read-back confirmation and the plug service's resend. */
static uint8_t s_cmd_tsn = 0x80;   /* own ZCL sequence for these frames */

uint8_t zigbee_plug_send_on_off(bool on)
{
    esp_zb_zcl_basic_cmd_t    dst;
    esp_zb_zcl_address_mode_t mode;
    fill_plug_dst(&dst, &mode);

    uint8_t tsn = s_cmd_tsn++;
    /* ZCL header: frame control = cluster-specific, client->server, default
       response disabled (the read-back is what confirms); TSN; command id. */
    static uint8_t frame[3];
    frame[0] = 0x01 | 0x10;
    frame[1] = tsn;
    frame[2] = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;

    esp_zb_apsde_data_req_t req = {
        .dst_addr_mode = (uint8_t)mode,
        .dst_addr      = dst.dst_addr_u,
        .dst_endpoint  = dst.dst_endpoint,
        .profile_id    = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id    = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .src_endpoint  = dst.src_endpoint,
        .asdu_length   = sizeof(frame),
        .asdu          = frame,
        .tx_options    = 0,        /* no APS ACK, hence no late retransmission */
        .radius        = 0,        /* stack default */
    };
    esp_err_t err = esp_zb_aps_data_request(&req);
    ESP_LOGI(TAG, "CMD %-3s -> 0x%04x ep%d (tsn %u, %s addressing, no APS ack)%s",
             on ? "ON" : "OFF", s_plug_addr, s_plug_ep, tsn,
             s_have_ieee ? "IEEE" : "short", err == ESP_OK ? "" : " — send failed");
    return tsn;
}

uint8_t zigbee_plug_read_on_off(void)
{
    uint16_t attr = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
    esp_zb_zcl_read_attr_cmd_t cmd = {
        .clusterID   = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .direction   = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .attr_number = 1,
        .attr_field  = &attr,
    };
    fill_plug_dst(&cmd.zcl_basic_cmd, &cmd.address_mode);
    uint8_t tsn = esp_zb_zcl_read_attr_cmd_req(&cmd);
    ESP_LOGD(TAG, "READ OnOff -> 0x%04x ep%d (tsn %u)", s_plug_addr, s_plug_ep, tsn);
    return tsn;
}

/* Runs as a scheduler alarm, i.e. in the Zigbee task. */
static void network_up_cb(uint8_t param)
{
    (void)param;
    if (s_listener && s_listener->on_network_up) s_listener->on_network_up();
}

/* Only frames from the paired plug's On/Off cluster are of interest. */
static bool from_plug(const esp_zb_zcl_addr_t *src, uint8_t src_ep, uint16_t cluster)
{
    return s_plug_addr != 0xFFFF &&
           cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
           src_ep == s_plug_ep &&
           src->u.short_addr == s_plug_addr;
}

static bool attr_on_off(const esp_zb_zcl_attribute_t *attr, bool *on)
{
    if (attr->id != ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID ||
        attr->data.type != ESP_ZB_ZCL_ATTR_TYPE_BOOL || !attr->data.value)
        return false;
    *on = *(const uint8_t *)attr->data.value != 0;
    return true;
}

static esp_err_t zcl_action_handler(esp_zb_core_action_callback_id_t id, const void *message)
{
    if (!s_listener) return ESP_OK;

    if (id == ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID) {
        const esp_zb_zcl_cmd_read_attr_resp_message_t *m = message;
        if (!from_plug(&m->info.src_address, m->info.src_endpoint, m->info.cluster))
            return ESP_OK;
        uint8_t tsn = m->info.header.tsn;
        bool ok = false, on = false;
        if (m->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
            for (const esp_zb_zcl_read_attr_resp_variable_t *v = m->variables; v; v = v->next) {
                if (v->status == ESP_ZB_ZCL_STATUS_SUCCESS && attr_on_off(&v->attribute, &on)) {
                    ok = true;
                    break;
                }
            }
        }
        ESP_LOGD(TAG, "READ resp tsn %u: %s", tsn, ok ? (on ? "ON" : "OFF") : "failed");
        if (s_listener->on_read_result) s_listener->on_read_result(tsn, ok, on);
    } else if (id == ESP_ZB_CORE_REPORT_ATTR_CB_ID) {
        const esp_zb_zcl_report_attr_message_t *m = message;
        bool on;
        if (from_plug(&m->src_address, m->src_endpoint, m->cluster) &&
            attr_on_off(&m->attribute, &on)) {
            ESP_LOGI(TAG, "REPORT OnOff: %s", on ? "ON" : "OFF");
            if (s_listener->on_report) s_listener->on_report(on);
        }
    }
    return ESP_OK;
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
    /* Under the stack lock: the Zigbee task reads these while addressing and
       filtering, and the plug service must stop in the same step. */
    if (s_started) esp_zb_lock_acquire(portMAX_DELAY);
    s_plug_addr = 0xFFFF;
    s_plug_ep   = 0xFF;
    s_have_ieee = false;
    /* Without a running stack there is nothing to stop, and the listener
       would touch the Zigbee scheduler. */
    if (s_started && s_listener && s_listener->on_unpaired) s_listener->on_unpaired();
    if (s_started) esp_zb_lock_release();

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, NVS_KEY_ADDR);
        nvs_erase_key(nvs, NVS_KEY_EP);
        nvs_erase_key(nvs, NVS_KEY_IEEE);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Plug unpaired");
    return ESP_OK;
}
