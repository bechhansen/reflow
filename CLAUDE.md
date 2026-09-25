# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DIY reflow hotplate controller. An old clothes iron (inverted, soleplate up) is the heat source. An ESP32-C6 pairs with a Zigbee smart wall plug, reads temperature from an MLX90614ESF IR sensor over I2C, and serves a real-time web UI over Wi-Fi.

**The reflow control algorithm has been removed.** The firmware currently reads and reports temperature, manages reflow profiles, and handles Zigbee pairing and Wi-Fi. Nothing drives the relay: `reflow_ctrl_start()` and `reflow_ctrl_stop()` are empty stubs, kept so the UI's Start and Stop buttons remain wired to something that exists.

## Repository Structure

```
Reflow/
├── controller/          ESP-IDF firmware project (esp32c6)
│   ├── main/            All application source (.c/.h)
│   ├── spiffs_image/    SPIFFS filesystem image (flashed alongside firmware)
│   │   ├── www/         Web UI (index.html)
│   │   └── profiles/    Default reflow profiles (.json)
│   ├── partitions.csv   Custom partition table (factory 1.5 MB + SPIFFS 512 KB)
│   └── sdkconfig.defaults  Baseline sdkconfig (Zigbee coordinator, SPIFFS, WebSocket)
├── hardware/
│   ├── enclosure/       3D-printed ESP32-C6 board enclosure
│   ├── iron_stand/      3D-printed cradle for inverted iron
│   └── sensor_bracket/  3D-printed MLX90614 mount above soleplate
├── REQUIREMENTS.md      Uniquely-keyed requirements (REQ-HW/FW/NET/WEB-xxx)
└── CLAUDE.md
```

## Build & Flash Commands

```bash
# Source ESP-IDF 6.1 (installed at ~/esp/esp-idf-v6.1)
. ~/esp/esp-idf-v6.1/export.sh

cd controller

# First-time setup
idf.py set-target esp32c6

# Configure Wi-Fi SSID/password and I2C pins (optional — can also be set via NVS at runtime)
idf.py menuconfig   # → Reflow Controller

# Build (also generates SPIFFS image from spiffs_image/)
idf.py build

# Flash firmware + SPIFFS image
idf.py flash

# Monitor serial output
idf.py monitor

# Build + flash + monitor in one step
idf.py flash monitor
```

## Firmware Architecture

All source lives in `controller/main/`. Key modules:

| File | Responsibility |
|------|---------------|
| `main.c` | `app_main()`: init sequence, boot log |
| `temperature.c/h` | MLX90614 I2C driver (register 0x07, SMBus read) |
| `zigbee_plug.c/h` | Zigbee transport: coordinator task, NVS pairing, raw On/Off command + OnOff attribute read, listener callbacks |
| `plug_fsm.c/h` | Pure plug state machine (pending / confirm-by-read / retry / Unknown). No ESP-IDF deps; host-tested |
| `plug_ctrl.c/h` | Plug service: the only thing that switches the plug. Wraps `plug_fsm` in Zigbee context, publishes a lock-free status snapshot |
| `reflow_curve.h` | Waypoint table types. No ESP-IDF deps |
| `reflow_profile.c/h` | JSON profile load/save from SPIFFS, on top of `reflow_curve` |
| `reflow_ctrl.c/h` | Sensor shell: polls the MLX90614 for telemetry, holds run state. Start/stop are empty stubs |
| `web_server.c/h` | HTTP server, WebSocket broadcast, REST API |
| `wifi_manager.c/h` | STA with AP fallback, NVS credential storage |

### Control Algorithm

Removed. `reflow_algo.c/h` (model feedforward + PI trim with a delta-sigma
output stage) and the `controller/sim/` scoring harness have both been deleted.
`reflow_ctrl.c` is now a sensor shell: it polls the MLX90614 every 500 ms so
temperature telemetry keeps flowing, and holds the run state the UI reads.

Nothing temperature-driven commands the plug. A future control algorithm
should switch it only through `plug_ctrl_set()`, check `plug_ctrl_get_status()`
for the confirmed state, and subscribe with `plug_ctrl_add_result_cb()`.

### Zigbee

The Zigbee stack runs in its own FreeRTOS task (`esp_zb_stack_main_loop()`). All calls to `esp_zb_*` APIs from other tasks must use `esp_zb_lock_acquire(portMAX_DELAY)` / `esp_zb_lock_release()`. The device acts as a coordinator (`ESP_ZB_ZC_CONFIG()`). Pairing opens a permit-join window; the `ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE` handler calls `esp_zb_zdo_find_on_off_light()` to identify the plug endpoint.

### Plug service

`plug_ctrl.c` is the only code that switches the plug (UI test toggle, console
`plug on|off|toggle|status`, and in future a control algorithm). A change is
**pending** until a Read Attributes response for OnOff reports the target
value. Requests made while a change is pending are rejected (`ESP_ERR_INVALID_STATE`).
Retry policy: 200 ms settle, then a read every 200 ms for up to 10 reads (~2 s).
A reply to any read of the sequence counts. An old-state reply is not a
mismatch until the sequence runs out; then the command is resent once. With no
reply at all the state becomes `PLUG_UNKNOWN`. Polling: every 10 s while
idle, every 1 s while Unknown. Measured: 0.5–1.5 s from command to confirmation.

Why reads are so frequent: Wi-Fi owns the shared radio most of the time, and
the plug's replies are reliably received only right after we transmit. Raising
the 802.15.4 coexistence priority did not help and can kill Wi-Fi; the
measurements are recorded in `zigbee_plug.c`. Do not shorten the settle or the
interval without re-measuring: 50 ms settle and a 100 ms interval were both
worse.
The decision logic lives in the pure `plug_fsm.c`; test it on the host with
`make -C controller/test/plug_fsm test`.

The FSM is only touched in Zigbee context (listener callbacks and scheduler
alarms run in the Zigbee task; public calls take `esp_zb_lock`). Readers
elsewhere use the spinlock-guarded snapshot from `plug_ctrl_get_status()`.

### WebSocket

`web_server_broadcast(json)` sends to all connected clients via `httpd_ws_send_frame_async()`. Client fds are tracked in `s_ws_fds[MAX_WS_CLIENTS]` guarded by a mutex. A 2 Hz `esp_timer` fires `telemetry_cb()` which builds and broadcasts the JSON telemetry frame.

### Wi-Fi

On boot: load credentials from NVS (namespace `wifi_cfg`). If none or connection fails within 10 s, start AP (`Reflow-Setup`, open). The web UI's Wi-Fi settings page posts new credentials to `/api/wifi/credentials` which saves to NVS and reboots.

## Profile Format

JSON with a `waypoints` array. The controller linearly interpolates between waypoints:

```json
{
  "name": "SAC305 Lead-Free",
  "description": "...",
  "waypoints": [
    { "time": 0,   "temp": 25,  "label": "Start"   },
    { "time": 60,  "temp": 150, "label": "Preheat" },
    { "time": 180, "temp": 180, "label": "Soak"    },
    { "time": 202, "temp": 245, "label": "Reflow"  },
    { "time": 217, "temp": 245, "label": "Peak"    },
    { "time": 320, "temp": 50,  "label": "Cool"    }
  ]
}
```

Profiles are stored in SPIFFS at `/spiffs/profiles/<name>.json`. Default profiles (SAC305, Sn63Pb37) are pre-loaded from `spiffs_image/profiles/`.

## WebSocket API

**Device → browser (2 Hz telemetry):**
```json
{ "type":"telemetry", "temp":142.5, "ambient":31.2, "state":"idle",
  "sensor_ok":true, "plug_paired":true, "plug_available":true,
  "plug_state":"off", "plug_pending":false, "profile":"",
  "wifi_mode":"sta", "ip":"192.168.1.42", "hostname":"reflow" }
```
`plug_state` is `"unknown" | "off" | "on"`, and is always a state the plug confirmed.

**Device → browser (plug change finished, or `plug_toggle` rejected — the latter to the sender only):**
```json
{ "type":"plug_result", "ok":false, "state":"unknown", "reason":"timeout" }
```
`reason`: `timeout`, `mismatch`, `unpaired` (finished) or `busy`, `unknown`, `not_paired`, `unavailable` (rejected).

**Browser → device:**
```json
{ "type": "start", "profile": "SAC305 Lead-Free" }
{ "type": "stop" }
{ "type": "start_pairing" }
{ "type": "unpair" }
{ "type": "plug_toggle" }
```

## Hardware Defaults (configurable via `idf.py menuconfig`)

| Parameter | Default |
|-----------|---------|
| I2C SDA   | GPIO 6  |
| I2C SCL   | GPIO 7  |
| Zigbee endpoint | 10 |
| Wi-Fi SSID | *(empty — AP mode)* |
