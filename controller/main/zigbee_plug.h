#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* Zigbee transport for the paired On/Off plug: network, pairing, and the raw
   On/Off command and OnOff attribute read. Deciding WHAT to send and when a
   change is confirmed belongs to plug_ctrl.c, not here. */

typedef void (*zigbee_pairing_cb_t)(bool success, uint16_t short_addr, uint8_t endpoint);
typedef void (*zigbee_countdown_cb_t)(uint8_t remaining);

/* Events for the plug service. Every callback runs in the Zigbee task, with
   the stack lock effectively held: call the zigbee_plug_send_* functions
   directly, never esp_zb_lock_acquire(). */
typedef struct {
    void (*on_network_up)(void);      /* coordinator running and settled */
    void (*on_paired)(void);
    void (*on_unpaired)(void);
    void (*on_read_result)(uint8_t tsn, bool ok, bool on);
    void (*on_report)(bool on);
} zigbee_plug_listener_t;

/* Must be set before zigbee_plug_init(). */
void      zigbee_plug_set_listener(const zigbee_plug_listener_t *listener);

esp_err_t zigbee_plug_init(void);
bool      zigbee_plug_is_paired(void);
esp_err_t zigbee_plug_start_pairing(uint8_t duration_s, zigbee_pairing_cb_t cb, zigbee_countdown_cb_t countdown_cb);
esp_err_t zigbee_plug_unpair(void);

/* Zigbee context only (a listener callback, a scheduler alarm, or with
   esp_zb_lock held). Both return the ZCL transaction sequence number. That is
   NOT a delivery result: only a read response says what the plug really did. */
uint8_t   zigbee_plug_send_on_off(bool on);
uint8_t   zigbee_plug_read_on_off(void);
