# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DIY reflow hotplate controller. An old clothes iron (inverted, soleplate up) is the heat source. An ESP32-C6 coordinates a Zigbee smart wall plug to toggle heat on/off, reads temperature from an MLX90614ESF IR sensor over I2C, and serves a real-time web UI over Wi-Fi.

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
# Source ESP-IDF (installed at ~/esp/esp-idf)
. ~/esp/esp-idf/export.sh

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
| `zigbee_plug.c/h` | Zigbee coordinator task, on/off commands, NVS pairing |
| `reflow_profile.c/h` | JSON profile load/save from SPIFFS, linear interpolation |
| `reflow_ctrl.c/h` | PID control loop + relay cycle task |
| `web_server.c/h` | HTTP server, WebSocket broadcast, REST API |
| `wifi_manager.c/h` | STA with AP fallback, NVS credential storage |

### Control Algorithm

Time-proportional PID with an 8-second cycle. The relay has a ≥ 2 s minimum switching period, giving 5 valid duty levels (0 / 25 / 50 / 75 / 100 %). A `relay_cycle_task` runs continuously; a `run_task` is spawned per reflow run and self-deletes on completion. Force 100 % when setpoint − temp > 10 °C; hand off to PID within 10 °C.

### Zigbee

The Zigbee stack runs in its own FreeRTOS task (`esp_zb_stack_main_loop()`). All calls to `esp_zb_*` APIs from other tasks must use `esp_zb_lock_acquire(portMAX_DELAY)` / `esp_zb_lock_release()`. The device acts as a coordinator (`ESP_ZB_ZC_CONFIG()`). Pairing opens a permit-join window; the `ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE` handler calls `esp_zb_zdo_find_on_off_light()` to identify the plug endpoint.

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
{ "type":"telemetry", "temp":142.5, "setpoint":145.0, "elapsed":45, "phase":"Soak",
  "state":"running", "plug_paired":true, "plug_on":true, "duty_steps":3,
  "wifi_mode":"sta", "ip":"192.168.1.42" }
```

**Browser → device:**
```json
{ "type": "start", "profile": "SAC305 Lead-Free" }
{ "type": "stop" }
{ "type": "start_pairing" }
{ "type": "unpair" }
```

## Hardware Defaults (configurable via `idf.py menuconfig`)

| Parameter | Default |
|-----------|---------|
| I2C SDA   | GPIO 6  |
| I2C SCL   | GPIO 7  |
| Zigbee endpoint | 10 |
| Wi-Fi SSID | *(empty — AP mode)* |
