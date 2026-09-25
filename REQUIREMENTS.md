# Reflow Hotplate — Requirements

## Hardware (REQ-HW)
| Key        | Requirement |
|------------|-------------|
| REQ-HW-001 | MCU is ESP32-C6 (native 802.15.4 Zigbee + Wi-Fi) |
| REQ-HW-002 | Temperature sensor is MLX90614ESF (IR contactless, I2C/SMBus, 3.3 V, address 0x5A) |
| REQ-HW-003 | Heat source is an inverted clothes iron (soleplate up as hotplate) |
| REQ-HW-004 | Heat is controlled by toggling a Zigbee smart wall plug (binary on/off, mechanical relay) |
| REQ-HW-005 | Relay must not switch more than once per second (`MIN_DWELL_MS`). Wear is not otherwise constrained: measurement showed accuracy is limited by thermal overrun, not by switching rate |
| REQ-HW-006 | 3D-printed enclosure for the ESP32-C6 controller board |
| REQ-HW-007 | 3D-printed stand/cradle for the inverted iron |
| REQ-HW-008 | 3D-printed bracket positioning the MLX90614 above the soleplate |
| REQ-HW-009 | I2C bus requires 4.7 kΩ pull-up resistors to 3.3 V — one on SDA (GPIO 6), one on SCL (GPIO 7). Most MLX90614 breakout modules (GY-906: 4.7 kΩ, Adafruit: 10 kΩ) already fit them; a bare MLX90614ESF does not. ESP32-C6 dev boards carry none, and the MCU's internal pull-ups (~45 kΩ) are too weak to substitute |
| REQ-HW-010 | Sensor cable is 4-conductor stranded 0.25 mm² (24 AWG) unshielded PVC — Bedea LiYY 4×0,25 mm² or equivalent. Shielding is unnecessary: heat switching happens inside the remote Zigbee plug, so no mains runs alongside the sensor cable |
| REQ-HW-011 | Sensor cable cores per DIN 47100: white → 3V3, brown → GND, green → SDA, yellow → SCL |
| REQ-HW-012 | Sensor cable run kept under 1 m. Above ~1.5 m, pull-ups drop to 2.2 kΩ to stay within the 1 µs standard-mode I2C rise-time limit |
| REQ-HW-013 | Sensor bracket must hold the MLX90614 and its cable below 80 °C (cable PVC limit; sensor ambient limit is 85 °C) while the soleplate reaches 245 °C |

## Firmware (REQ-FW)
| Key        | Requirement |
|------------|-------------|
| REQ-FW-001 | Firmware built with ESP-IDF targeting esp32c6 |
| REQ-FW-002 | MLX90614 driver reads object temperature (reg 0x07) and ambient/die temperature (reg 0x06) via I2C at 100 kHz, PEC-verified; resolution 0.02 °C |
| REQ-FW-003 | Zigbee stack runs as coordinator; sends On/Off commands (cluster 0x0006) to the paired plug and reads its OnOff attribute (0x0000) back. All plug switching goes through the plug service (`plug_ctrl.c`) |
| REQ-FW-004 | Zigbee pairing opens a 60-second permit-join window. The plug's **IEEE (64-bit) address** is stored in NVS and used to address commands — the 16-bit short address is reassigned when a device rejoins, which silently orphaned the pairing |
| REQ-FW-005 | Unpair removes the NVS entry; controller blocks reflow start if no plug is paired |
| REQ-FW-006 | Control algorithm is **hysteresis on a predicted landing temperature**, evaluated at 1 Hz. Power off when `temp_f + inflight + lead × dT/dt` would exceed the setpoint by half the band; power on when `temp_f` falls half a band below it. Band is fixed at 2 °C (`BAND_C`) |
| REQ-FW-007 | Anticipation is proportional to energy delivered: `inflight = A_obs × min(on_time, L)`, and zero while power is off. A learned constant offset was wrong — it was fitted from long ramp pulses and then applied to short hold pulses, holding the plate ~5 °C below setpoint |
| REQ-FW-008 | Cooling segments force power off unconditionally; no active cooling exists |
| REQ-FW-009 | No PID. There is no integral term, so nothing to wind up while following a ramping setpoint |
| REQ-FW-010 | Reflow profiles stored as JSON in SPIFFS (`/spiffs/profiles/`) |
| REQ-FW-011 | Default profiles shipped in SPIFFS: `SAC305 Lead-Free`, `Sn63Pb37 Leaded`, `SMD291SNL SAC305` (Chip Quik paste, peak held to 235 °C), and `Calibration 170` (controller tuning, not for soldering) |
| REQ-FW-012 | Profile format uses time-temperature waypoints; controller interpolates linearly between them |
| REQ-FW-013 | Web UI static files served from SPIFFS (`/spiffs/www/`) |
| REQ-FW-014 | UART console: logging output with IP address, profile count, and error messages at boot |
| REQ-FW-015 | **A run's elapsed time always starts at 0 and always advances with real time.** The run clock is the profile's time base and must never be offset, skipped ahead, scaled, paused or resumed — whatever the plate's starting temperature. Pressing Start always begins the profile at its first waypoint |
| REQ-FW-016 | Temperature is low-pass filtered (EMA α=0.3) and `dT/dt` measured over a 5 s window; single readings differing by more than 20 °C are rejected as implausible (the sensor can return a wildly wrong value that still passes its PEC check) |
| REQ-FW-017 | The heat source is characterised during each run and persisted in NVS (`reflow_plant`): `R` heating rate, `L` dead time, `C` cooling rate. `R` is only re-measured from stretches where the controller genuinely demands full power, otherwise it records the profile's ramp rate rather than the plant's capability. `POST /api/model/reset` returns to the built-in baseline |
| REQ-FW-018 | Lead term is `1.0 × L`, clamped to 1–10 s; setpoint look-ahead is `L` clamped to 1–5 s. Both bounds were chosen by simulation across plants with dead times from 0.5 s to 40 s |
| REQ-FW-019 | **Safety aborts.** A run stops, and power is cut, if: the plate rises < 5 °C after 45 s of accumulated power while `dT/dt < 0.05 °C/s` (heater not connected, or sensor misaimed); or the sensor returns 15 identical readings while heating (stale reading — heating blind). A stall *at temperature* (full power, 30 s, no rise) warns but continues, since it is usually the heat source's own thermostat |
| REQ-FW-020 | A paired plug is commanded OFF ~3 s after the Zigbee coordinator starts, on every boot, and the OFF is confirmed by read-back like any other change. The plug's state across a restart is never assumed |
| REQ-FW-021 | **A plug change is done only when confirmed.** After an On/Off command the controller waits 200 ms, then reads the OnOff attribute every 200 ms. The change is done only when a read reports the requested value. The command's return value (a ZCL sequence number) is never treated as confirmation. A reply to any read of the current sequence is accepted, since under Wi-Fi coexistence replies often arrive after the next read went out. Replies from earlier sequences are ignored |
| REQ-FW-022 | **No overlapping changes.** While a change is pending, any further request is rejected (not queued) and nothing is sent |
| REQ-FW-023 | Retry policy: a read sequence is up to 10 reads, 200 ms apart. An old-state reply does not end the sequence. If the sequence runs out after the plug answered with only the old state, the command is resent once; if that also does not take effect, the request fails (`mismatch`) and the real state is kept. If the plug did not answer at all, the state becomes **Unknown** and the request fails (`timeout`) |
| REQ-FW-024 | While idle, the plug's OnOff attribute is polled every 10 s, and every 1 s while the state is Unknown, so the state recovers and follows manual switching at the plug. Unsolicited attribute reports update the state only while no change is pending |
| REQ-FW-025 | The 802.15.4 coexistence priorities stay at the ESP-IDF defaults. Raised priorities were measured to give no reliable Zigbee gain and to break or kill Wi-Fi on the single-radio ESP32-C6 |

## Networking (REQ-NET)
| Key        | Requirement |
|------------|-------------|
| REQ-NET-001 | On boot, device attempts Wi-Fi STA connection using NVS-stored credentials |
| REQ-NET-002 | If no credentials exist or STA connection fails within 15 s (`STA_TIMEOUT_MS`), device starts AP mode (SSID: `Reflow-Setup`, open) |
| REQ-NET-003 | If STA connection drops, device retries `STA_RETRY_MAX` times then falls back to AP mode |
| REQ-NET-004 | HTTP server provides REST API for profile CRUD: `GET /api/profiles`, `GET/POST/DELETE /api/profiles/{name}`. Profiles are addressed **by filename**, so a profile's `name` field should match its filename |
| REQ-NET-005 | WebSocket endpoint at `ws://device/ws` handles all real-time control and telemetry |
| REQ-NET-006 | WebSocket supports up to 4 simultaneous clients; messages broadcast to all connected clients |
| REQ-NET-007 | Telemetry broadcast at 2 Hz: `{ type, temp, ambient, state, sensor_ok, plug_paired, plug_available, plug_state, plug_pending, profile, hostname, wifi_mode, ip }`. `plug_state` is `unknown`, `off` or `on`, and is always a confirmed state |
| REQ-NET-008 | Pairing events broadcast over WebSocket: `pairing_open` (with countdown), `pairing_success`, `pairing_failed` |
| REQ-NET-009 | WebSocket control messages: `start`, `stop`, `start_pairing`, `unpair`, `plug_toggle`. A finished plug change is broadcast as `{type:"plug_result", ok, state, reason}`. A rejected `plug_toggle` is answered to the sender only, with `reason` set to `busy`, `unknown`, `not_paired` or `unavailable` |
| REQ-NET-010 | If the pre-connect scan finds zero networks, the STA bring-up is treated as failed and the device soft-restarts once per power cycle (RTC-flagged). The ESP32-C6 receiver intermittently comes up deaf after a cold boot; a chip-level restart clears it, a driver restart does not. Root cause unidentified |
| REQ-NET-011 | mDNS hostname is configurable and stored in NVS (`wifi_cfg/hostname`), default `reflow`, validated as a DNS label (a-z, 0-9, hyphen, max 32, no leading/trailing hyphen) |
| REQ-NET-012 | Further endpoints: `GET /api/wifi/scan`, `GET /api/wifi/config`, `POST /api/wifi/credentials` (accepts `ssid`/`password` and/or `hostname`, then reboots), `POST /api/model/reset` |
| REQ-NET-013 | Telemetry carries the name of the profile actually running (empty when idle) so the UI plots the correct target curve regardless of which client started the run |
| REQ-NET-014 | Broadcast enumerates httpd's own socket list and filters by `httpd_ws_get_fd_info()`. ESP-IDF 6.x does not invoke the URI handler for the WebSocket handshake, so a self-managed client list stays empty and telemetry reaches nobody |

## Web UI (REQ-WEB)
| Key        | Requirement |
|------------|-------------|
| REQ-WEB-001 | Three pages served from SPIFFS: `index.html` (dashboard), `network.html` (network settings), `profiles.html` (profile editor). Chart.js is bundled locally — the device has no internet access |
| REQ-WEB-002 | Responsive layout: single-column on mobile (< 768 px), two-column on large screens (≥ 768 px) |
| REQ-WEB-003 | Touch-friendly controls with minimum 44 px tap targets |
| REQ-WEB-004 | Live temperature graph using Chart.js: actual temperature (streaming) + profile target curve |
| REQ-WEB-005 | Graph X-axis = elapsed seconds, Y-axis = °C; responsive and fills its container |
| REQ-WEB-006 | Profile selector dropdown, Start and Stop buttons via WebSocket, elapsed time, current phase label |
| REQ-WEB-007 | Zigbee plug section: status badge (Not paired / Unknown / Switching… / On / Off, confirmed state only), Pair Plug button (60 s countdown overlay), Unpair button |
| REQ-WEB-015 | Zigbee plug section has a **Test toggle** button. It is disabled unless the plug is available and its state is known with no change pending, and from the click until the device responds. The outcome is shown as a toast |
| REQ-WEB-008 | `profiles.html`: list, create, edit, rename, delete and upload profiles. Waypoint table with live SVG preview, and derived peak / total time / max ramp rate / time above 217 °C. Validates against firmware limits (16 waypoints, strictly increasing times, filename-safe names) |
| REQ-WEB-009 | `network.html`: current mode/address/SSID/hostname, network scan with channel and signal, SSID/password entry, hostname entry with live `.local` preview. Must remain usable in AP mode, so it uses no WebSocket |
| REQ-WEB-010 | Dashboard shows a Control Model panel — band, lead, overrun, heating rate, dead time, cooling rate, relay switch count, measured duty — each with a tap/hover explanation, plus a button to forget the learned heat source |
| REQ-WEB-011 | Sensor ambient temperature is displayed, amber from 65 °C and red with a warning from 80 °C (MLX90614ESF is rated to 85 °C ambient) |
| REQ-WEB-012 | A heater-not-responding stall is surfaced as a banner on the dashboard, not only in the serial log |
| REQ-WEB-013 | Zigbee controls are disabled with an explanation while in AP mode, where Zigbee cannot run; the profile selector is locked while a run is in progress and re-enabled when it completes |
| REQ-WEB-014 | No blocking `alert()`/`confirm()` dialogs: notifications are inline toasts and confirmations a modal, so they never stall the telemetry socket |
