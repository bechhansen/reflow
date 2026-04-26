#pragma once
#include "esp_err.h"
#include <stdbool.h>

typedef void (*zigbee_pairing_cb_t)(bool success, uint16_t short_addr, uint8_t endpoint);
typedef void (*zigbee_countdown_cb_t)(uint8_t remaining);

esp_err_t zigbee_plug_init(void);
esp_err_t zigbee_plug_set(bool on);
bool      zigbee_plug_is_paired(void);
esp_err_t zigbee_plug_start_pairing(uint8_t duration_s, zigbee_pairing_cb_t cb, zigbee_countdown_cb_t countdown_cb);
esp_err_t zigbee_plug_unpair(void);
