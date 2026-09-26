# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DIY reflow hotplate controller. An old clothes iron (inverted, soleplate up) is the heat source. An ESP32-C6 pairs with a Zigbee smart wall plug, reads temperature from an MLX90614ESF IR sensor over I2C, and serves a real-time web UI over Wi-Fi.

The firmware follows a reflow profile with a model feedforward + PI(D) controller that switches the iron through the Zigbee plug (minimum 2 s dwell between switches). Cool-down is passive. Runs are started and stopped from the web UI or the serial console; the controller is tuned on the real hardware with `controller/tools/hil.py`.

## Repository Structure

```
Reflow/
├── controller/          ESP-IDF firmware project (esp32c6)
│   ├── main/            All application source (.c/.h)
│   ├── web/             Web UI, embedded (gzipped) in the firmware at build time:
│   │                    index.html, profiles.html, network.html, settings.html,
│   │                    theme.js (light/dark), units.js (°C/°F), chart.js
│   ├── profiles/        Default reflow profiles (.json), also embedded; written to
│   │                    SPIFFS only on a device that has no profiles at all
│   ├── tools/           hil.py (hardware-in-the-loop), embed_web.py (build step)
│   ├── partitions.csv   8 MB: two 2 MB OTA app slots, nvs, SPIFFS 512 KB (profiles), Zigbee
│   └── sdkconfig.defaults  Baseline sdkconfig (8 MB flash, rollback, -Os, Zigbee, WebSocket)
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

# Build (the web UI and default profiles are embedded in the app)
idf.py build

# Flash over USB: bootloader, partition table, otadata, app. Never SPIFFS, so
# saved profiles survive. After the first flash, updates go over Wi-Fi.
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
| `reflow_curve.c/h` | Waypoint table types and curve maths (interpolation, slope, cool-start). No ESP-IDF deps |
| `reflow_algo.c/h` | Pure control law: feedforward + PI(D), profile clock hold, coast guard, dwell-limited sigma-delta output, safety faults. No ESP-IDF deps; host-tested |
| `reflow_profile.c/h` | JSON profile load/save from SPIFFS, on top of `reflow_curve` |
| `reflow_ctrl.c/h` | Controller shell: 500 ms task that samples the MLX90614, steps `reflow_algo`, switches the plug, logs `@`-lines, holds run state, tuning params (NVS `ctrl`), run trace buffer |
| `web_server.c/h` | HTTP server (embedded web UI, ETag/no-cache), WebSocket broadcast, REST API |
| `ota_update.c/h` | Firmware updates over Wi-Fi: GitHub release check, install, upload, confirm/rollback |
| `wifi_manager.c/h` | STA with AP fallback, NVS credential storage |

### Control Algorithm

`reflow_algo.c` (pure, host-tested with `make -C controller/test/reflow_algo test`)
runs every 0.5 s against a first-order-plus-dead-time model of the iron,
`tau·dT/dt = K·u − (T − T_room)`:

- **Feedforward** for the curve `lookahead` s ahead (covers the dead time),
  plus **PI(D)** on the error now. Over-temperature error is weighted
  (`over_weight`); the integral is frozen while saturated, while the clock is
  held, and while the heater is forced off.
- **Real time**: the profile is followed on its own timeline, which keeps the run
  as close to the profile as possible (`max_hold` = 0, the default). Optionally
  (`max_hold` > 0) the profile clock may slow while the iron lags a non-falling
  segment by more than `lag_band` (stopping at `lag_band + lag_span`), for at
  most `max_hold` s per phase (`@E,…,hold_limit`), to keep a soak or peak from
  being cut short at the cost of a longer run. More than `max_stretch` s of hold
  in total faults the run (`stall`).
- **Coast guard**: where the curve peaks within the horizon, heat is cut once
  `T + slope·coast` would pass the peak.
- **Cool-down** is passive: past the last rise the heater is off and the run is
  COMPLETE when the curve ends; it never waits for any temperature.
- **Always the whole profile**: a run starts at the profile's time 0 whatever
  the iron's temperature; a hot iron gets no heat until the curve catches up.
- **Integral**: frozen while saturated, while the clock is held, and while the
  heater is forced off, but NOT while the coast guard holds it off at a
  plateau (an integrator blind to the above-target periods winds up; that was
  a +1.5 °C offset at holds).
- **Output**: sigma-delta on duty minus the plug state (last confirmed, or last
  commanded since: read replies are often lost, so Unknown is normal). Below
  `pulse_duty` (0.3), on stretches where the target moves less than half a
  pulse-bump per pulse cycle, pulses are instead timed on the temperature:
  fire when the reading predicted `theta` s ahead (2 s slope) falls half a
  bump below the target, so each bump is centred on it. The bump is heat per
  pulse (model rate × `min_on`) less the loss while it builds, or `pulse_rise`
  if set. Generic: curve shape and model only, no phase names. Measured:
  Sn63Pb37 soak p-p 8.3 → 4.9 °C. A change
  is commanded only after the state has held for `min_dwell`/`min_on` (2 s) and
  nothing is pending. The wanted state is re-sent `reassert` s after each change
  and every `refresh` s. If the heater is meant to be off but the iron still
  heats faster than `anomaly_slope`, OFF is sent at once (`@E,…,anomaly`).
  Unknown for more than `plug_timeout` is a fault. Faults (sensor, sensor_stale,
  overtemp, stall, plug) switch off immediately. A reading more than 20 °C from
  the last is rejected (the MLX90614 can return a bad value that passes PEC),
  and 15 identical readings while heating fault the run (`sensor_stale`:
  heating blind). The over-temperature limit is relative:
  the run's highest target (profile peak, hold or step temperature) plus
  `over_margin` (25 °C). There is no fixed ceiling in the code, so a hotter
  heat source can run a hotter profile.
- **Measured model** (compiled-in defaults, 2026-09-25): 2.84 °C/s gross at full
  power, heat loss ~quadratic (~0.38 °C/s at 200 °C, `loss_quad` 0.12 with a
  pinned linear term), ~2 s element-to-soleplate lag, no real dead time. A 2 s
  pulse adds ~5 °C, which sets the hold ripple floor. This iron's own
  thermostat cuts out with the soleplate near 231 °C at reflow ramp rates (a
  property of this heat source, not a limit in the code). First profile run (SMD291SNL, older firmware that joined the curve at the iron's temperature): RMS
  1.6 °C, peak +2.8 °C; time above liquidus is long (106 s vs 55 s) because
  cool-down is passive at ~0.4 °C/s.

Parameters are `algo_params_t`, set at runtime with the console (`ctl set <key>
<val>`, `ctl save` to NVS, `ctl reset`). The controller switches the plug only
through `plug_ctrl_set()`; during a run the UI test toggle and console `plug`
commands are refused (`running`).

### Hardware-in-the-loop testing

`controller/tools/hil.py` (ESP-IDF Python env; pyserial, stdlib only) drives
the console and records the controller's `@T/@E/@S/@P` lines (format in
`reflow_ctrl.h`) to `controller/tools/logs/`:

```bash
python controller/tools/hil.py step 1.0 300 200 --duration 600  # open-loop step
python controller/tools/hil.py fit controller/tools/logs/<step>.log  # K, tau, theta, SIMC gains
python controller/tools/hil.py hold 150 --duration 600
python controller/tools/hil.py run "Calibration 170"
python controller/tools/hil.py analyze controller/tools/logs/<run>.log  # metrics + SVG chart
python controller/tools/hil.py cmd "ctl show"
```

Close `idf.py monitor` first. Ctrl-C sends `stop`. Console commands:
`start <profile>`, `step <duty> <secs> <maxT>`, `hold <temp>`, `stop`,
`status`, `ctl show|save|reset`, `ctl set <key> <val>`, `log on|off`.

### Zigbee

The Zigbee stack runs in its own FreeRTOS task (`esp_zb_stack_main_loop()`). All calls to `esp_zb_*` APIs from other tasks must use `esp_zb_lock_acquire(portMAX_DELAY)` / `esp_zb_lock_release()`. The device acts as a coordinator (`ESP_ZB_ZC_CONFIG()`). Pairing opens a permit-join window; the `ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE` handler calls `esp_zb_zdo_find_on_off_light()` to identify the plug endpoint.

### Plug service

`plug_ctrl.c` is the only code that switches the plug (UI test toggle, console
`plug on|off|toggle|status`, and the reflow controller). A change is
**pending** until a Read Attributes response for OnOff reports the target
value. Requests made while a change is pending are rejected (`ESP_ERR_INVALID_STATE`).
Retry policy: 200 ms settle, then a read every 200 ms for up to 10 reads (~2 s).
A reply to any read of the sequence counts. An old-state reply is not a
mismatch until the sequence runs out; then the command is resent once. With no
reply at all the state becomes `PLUG_UNKNOWN`. Polling: every 10 s while
idle, every 1 s while Unknown. Measured: 0.5–1.5 s from command to confirmation.

On/Off commands are sent as raw APS frames **without APS acknowledgement**
(`zigbee_plug_send_on_off()`). With the stack's APS ACK, lost ACKs made the
stack retransmit the command 1–3 s later: an OFF sent ~2.5 s after an ON was
confirmed, then the late ON switched the relay back on (measured: 14 s of
full-power heating under a confirmed OFF). The read-back confirmation and
resends cover a command that is lost outright.

Why reads are so frequent: Wi-Fi owns the shared radio most of the time, and
the plug's replies are reliably received only right after we transmit. Raising
the 802.15.4 coexistence priority did not help and can kill Wi-Fi; the
measurements are recorded in `zigbee_plug.c`. Do not shorten the settle or the
interval without re-measuring: 50 ms settle and a 100 ms interval were both
worse.
The decision logic lives in the pure `plug_fsm.c`; test it on the host with
`make -C controller/test/plug_fsm test`.

After boot the controller keeps re-sending OFF until the plug confirms it (the
plug service's own boot OFF can be lost, leaving the state Unknown with the
relay possibly still on from before the reboot).

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

Profiles are stored in SPIFFS at `/spiffs/profiles/<name>.json`. The defaults in `controller/profiles/` are embedded in the firmware and written only when SPIFFS has no profiles at all (a new device); existing profiles are never touched.

## Firmware updates (OTA) and releases

Two 2 MB app slots (`ota_0`, `ota_1`). An update is written into the idle slot
while the running one is untouched, verified, and booted. The new firmware
confirms itself after serving the web UI for 30 s, or at once when another
update is requested from the web UI; if it resets before confirming, the
bootloader returns to the previous slot (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`).
nvs, spiffs and zb_* never move, so updates keep Wi-Fi, tuning, units,
profiles and the Zigbee pairing.

- **GitHub**: `ota_update.c` checks `https://api.github.com/repos/<REFLOW_OTA_REPO>/releases/latest`
  60 s after boot, every 24 h and on request (STA mode). GitHub's "latest"
  excludes pre-releases, and the device also takes only a tag that is exactly
  `vX.Y.Z`. It offers the release's `reflow-controller.bin`; nothing installs
  until the user clicks Install. Needs the repository to be public.
- **Upload**: `POST /api/ota/upload` with a `.bin` body, any build of this
  project (checked by `project_name`), including alpha/beta.
- **Safety**: refused while a run is active; a run is refused while an update
  installs (`updating`); the heater is switched off and confirmed first.
- **REST**: `GET /api/ota` (state, version, build date, latest, progress,
  last check, message), `POST /api/ota/check`, `POST /api/ota/install`,
  `POST /api/ota/upload`. Telemetry adds `ota_state`, `ota_latest`, `ota_progress`.
- **Test options** (Kconfig, never in a release): `REFLOW_OTA_CHECK_URL`
  (a stand-in server; with `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP` for plain HTTP)
  and `REFLOW_OTA_TEST_NO_CONFIRM` (a build that never confirms, to test rollback).

**Releasing**: tag `main` and push the tag, e.g. `git tag v1.2.0 && git push origin v1.2.0`.
The workflow (`.github/workflows/esp32-build.yml`) runs the host tests and
builds; the firmware version is the tag (`controller/version.txt`, written by
CI). `vX.Y.Z` makes a production release, which the controllers offer as an update;
`vX.Y.Z-alpha*/-beta*/-rc*` makes a pre-release for manual upload only. Every
other push is built and kept as a 30-day artifact. Assets: `reflow-controller.bin`
(OTA/upload) and `reflow-<version>-usb.zip` (first-time or recovery USB flash,
with `FLASH.txt`). Locally, a changed `version.txt` needs `idf.py reconfigure`.

## WebSocket API

**Device → browser (2 Hz telemetry):**
```json
{ "type":"telemetry", "temp":142.5, "ambient":31.2, "state":"running",
  "sensor_ok":true, "plug_paired":true, "plug_available":true,
  "plug_state":"off", "plug_pending":false, "profile":"SAC305 Lead-Free",
  "phase":"Soak", "elapsed":95.5, "profile_t":92.0, "setpoint":158.1,
  "power":0.42, "fault":"", "run_id":3,
  "wifi_mode":"sta", "ip":"192.168.1.42", "hostname":"reflow" }
```
`state` is `idle | running | cooling | complete | error`. The run fields
(`phase` … `run_id`) are present only when not idle; `profile` is `"step"` or
`"hold"` for test runs, `""` when idle. `fault` is `sensor | overtemp | stall |
plug` in the error state.
`plug_state` is `"unknown" | "off" | "on"`, and is always a state the plug confirmed.

**Device → browser (start refused — to the sender only):**
```json
{ "type":"run_result", "ok":false, "reason":"busy" }
```
`reason`: `busy`, `sensor`, `plug`, `profile`, `not_found`.

`GET /api/run/trace` returns `{"run_id":3,"points":[[elapsed,temp,setpoint,heater,power],...]}`
(heater: confirmed 1/0, or -1 unknown; power: requested duty 0..1), the
current or last run's trace, plotted against real elapsed time, so
the UI can backfill the chart when opened mid-run. The chart shows the profile
(dashed) and the measured temperature against real time, the phase names
along the top (the current one bold), and below it a heater strip: grey
blocks where the heater was confirmed on, with the requested power as a line.
Everything is distinguishable without colour (pattern, fill, weight).

**Theme**: every page follows the browser's light/dark setting; the theme
button overrides it per browser (`localStorage`), and choosing what the browser
prefers returns to following it (`theme.js`, CSS variables with a dark set).

**Temperature unit**: `GET/POST /api/settings` `{"temp_unit":"C"|"F"}`,
device-wide in NVS namespace `ui` (default °C), also in telemetry as
`temp_unit`. Only the web pages convert (`units.js`); firmware, stored
profiles, console and logs are always °C. A page whose unit changes elsewhere
reloads.

**Device → browser (plug change finished, or `plug_toggle` rejected — the latter to the sender only):**
```json
{ "type":"plug_result", "ok":false, "state":"unknown", "reason":"timeout" }
```
`reason`: `timeout`, `mismatch`, `unpaired` (finished) or `busy`, `unknown`, `not_paired`, `unavailable`, `running` (rejected).

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
