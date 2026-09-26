# Reflow Hotplate — Requirements

## Hardware (REQ-HW)
| Key        | Requirement |
|------------|-------------|
| REQ-HW-001 | MCU is ESP32-C6 (native 802.15.4 Zigbee + Wi-Fi) with **8 MB flash**, for two 2 MB firmware slots (over-the-air updates) |
| REQ-HW-002 | Temperature sensor is MLX90614ESF (IR contactless, I2C/SMBus, 3.3 V, address 0x5A) |
| REQ-HW-003 | Heat source is an inverted clothes iron (soleplate up as hotplate) |
| REQ-HW-004 | Heat is controlled by toggling a Zigbee smart wall plug (binary on/off, mechanical relay) |
| REQ-HW-005 | The relay is never switched sooner than 2 s after its last change (`min_dwell`, and `min_on` after switching on), and never while a change awaits confirmation. The 2 s limit sets the pulse size, and so the ripple on flat profile stretches (~±2.5 °C on the development iron) |
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
| REQ-FW-006 | Control law (`reflow_algo.c`, pure C, host-tested) runs every 0.5 s. **Feedforward** from a first-order model of the heat source (`tau·dT/dt = K·u − (d + loss_quad·d²)`, d = T − T_room) for the profile `lookahead` s ahead, plus **PI** on the error now. Over-temperature error is weighted (`over_weight`) |
| REQ-FW-007 | **Coast guard**: where the profile tops out within the next `lookahead + coast` s, heat is cut while `T + slope·coast` would pass that peak, so heat stored in the iron does not overshoot. **Pulse timing**: below `pulse_duty`, on stretches where the target moves less than half a pulse-bump per pulse cycle, each minimum pulse is fired when the temperature predicted `theta` s ahead falls half a bump below the target, centring the bumps on it. Generic: uses the curve shape and model only, never phase names |
| REQ-FW-008 | Cooling segments force power off; there is no active cooling. The run is COMPLETE when the profile ends, whatever the plate temperature |
| REQ-FW-009 | The integral is frozen while the output is saturated in the error's direction, beyond ±15 °C error, while the profile clock is held, and while the heater is forced off (cool-down, faults), but not while the coast guard holds it off at a plateau: an integrator blind to above-target periods winds up |
| REQ-FW-010 | Reflow profiles stored as JSON in SPIFFS (`/spiffs/profiles/`) |
| REQ-FW-011 | Default profiles `SAC305 Lead-Free`, `Sn63Pb37 Leaded`, `SMD291SNL SAC305` (Chip Quik paste, peak 235 °C) and `Calibration 170` (tuning, not for soldering) are embedded in the firmware and written to SPIFFS only when it holds no profiles at all; existing profiles are never touched |
| REQ-FW-012 | Profile format uses time-temperature waypoints; controller interpolates linearly between them |
| REQ-FW-013 | Web UI static files are embedded in the firmware (gzipped, `controller/web/`), served with an ETag of the firmware build and `no-cache`, so an update carries the UI and the browser never runs old UI code against new firmware |
| REQ-FW-014 | UART console: logging output with IP address, profile count, and error messages at boot |
| REQ-FW-015 | **A run's elapsed time always starts at 0 and always advances with real time.** The run clock is the profile's time base and must never be offset, skipped ahead, scaled, paused or resumed — whatever the plate's starting temperature. Pressing Start always begins the profile at its first waypoint |
| REQ-FW-016 | Temperature is low-pass filtered (`filt_tau`, 1 s) and `dT/dt` measured by regression over 6 s (2 s for pulse timing). A reading more than 20 °C from the previous one is rejected as implausible (the sensor can return a wildly wrong value that still passes its PEC check); if readings stay missing for 2 s the run faults (`sensor`) |
| REQ-FW-017 | The heat source model (`K`, `tau`, `theta`, `loss_quad`, `t_room`) and the gains are parameters in NVS (`ctrl`), set with the console (`ctl set/save/reset`) from a step-test fit (`controller/tools/hil.py fit`). Compiled-in defaults are the model measured on the development iron |
| REQ-FW-018 | The profile clock runs in real time (`max_hold` = 0). Optionally (`max_hold` > 0) it may slow while the plate lags a non-falling segment by more than `lag_band`, for at most `max_hold` s per segment |
| REQ-FW-019 | **Faults** switch the heater off at once and end the run: sensor readings missing for 2 s (`sensor`); 15 identical readings while heating (`sensor_stale`: heating blind); temperature above the run's highest target + `over_margin` (25 °C) (`overtemp`, relative, no fixed ceiling); plug Unknown for 60 s (`plug`); profile clock held more than `max_stretch` s in total (`stall`, only with `max_hold` > 0) |
| REQ-FW-020 | A paired plug is commanded OFF ~3 s after the Zigbee coordinator starts, on every boot, and the OFF is confirmed by read-back like any other change. The plug's state across a restart is never assumed |
| REQ-FW-021 | **A plug change is done only when confirmed.** After an On/Off command the controller waits 200 ms, then reads the OnOff attribute every 200 ms. The change is done only when a read reports the requested value. The command's return value (a ZCL sequence number) is never treated as confirmation. A reply to any read of the current sequence is accepted, since under Wi-Fi coexistence replies often arrive after the next read went out. Replies from earlier sequences are ignored |
| REQ-FW-022 | **No overlapping changes.** While a change is pending, any further request is rejected (not queued) and nothing is sent |
| REQ-FW-023 | Retry policy: a read sequence is up to 10 reads, 200 ms apart. An old-state reply does not end the sequence. If the sequence runs out after the plug answered with only the old state, the command is resent once; if that also does not take effect, the request fails (`mismatch`) and the real state is kept. If the plug did not answer at all, the state becomes **Unknown** and the request fails (`timeout`) |
| REQ-FW-024 | While idle, the plug's OnOff attribute is polled every 10 s, and every 1 s while the state is Unknown, so the state recovers and follows manual switching at the plug. Unsolicited attribute reports update the state only while no change is pending |
| REQ-FW-025 | The 802.15.4 coexistence priorities stay at the ESP-IDF defaults. Raised priorities were measured to give no reliable Zigbee gain and to break or kill Wi-Fi on the single-radio ESP32-C6 |
| REQ-FW-026 | On/Off commands are sent as raw APS frames **without APS acknowledgement**: with it, a lost ACK made the stack retransmit an ON after a later OFF and switch the relay back on under a confirmed OFF |
| REQ-FW-027 | The controller re-sends the wanted plug state 2.5 s after each change and every 4 s after that, and forces OFF at once if the plate still heats faster than 1 °C/s 6 s after switching off (heating while off). It keeps re-sending OFF after boot, and after every run, until the plug confirms it |
| REQ-FW-028 | **Firmware updates**: two app slots; an update is written into the idle slot, verified and booted. The new firmware confirms itself after serving the web UI for 30 s (or at once when another update is requested); otherwise the bootloader rolls back to the previous one |
| REQ-FW-029 | Update sources: the latest GitHub release of `REFLOW_OTA_REPO` (checked 60 s after boot, every 24 h and on request; only a tag that is exactly `vX.Y.Z`, offered when newer than the running version, where a pre-release such as `vX.Y.Z-beta2` is older than `vX.Y.Z`; installed only on the user's request), or a `.bin` uploaded from the browser (any build of this project, including alpha/beta). Never while a run is active; a run cannot start during an update; the heater is switched off first and the update waits for the plug to confirm it, but an unreachable plug does not block an update |

## Networking (REQ-NET)
| Key        | Requirement |
|------------|-------------|
| REQ-NET-001 | On boot, device attempts Wi-Fi STA connection using NVS-stored credentials |
| REQ-NET-002 | If no credentials exist or STA connection fails within 15 s (`STA_TIMEOUT_MS`), device starts AP mode (SSID: `Reflow-Setup`, open) |
| REQ-NET-003 | If STA connection drops, device retries `STA_RETRY_MAX` times then falls back to AP mode |
| REQ-NET-004 | HTTP server provides REST API for profile CRUD: `GET /api/profiles`, `GET/POST/DELETE /api/profiles/{name}`. Profiles are addressed **by filename**, so a profile's `name` field should match its filename |
| REQ-NET-005 | WebSocket endpoint at `ws://device/ws` handles all real-time control and telemetry |
| REQ-NET-006 | WebSocket supports up to 4 simultaneous clients; messages broadcast to all connected clients |
| REQ-NET-007 | Telemetry broadcast at 2 Hz: `{ type, temp, ambient, state, sensor_ok, plug_paired, plug_available, plug_state, plug_pending, profile, [phase, elapsed, profile_t, setpoint, power, fault, run_id], temp_unit, ota_state, [ota_latest, ota_progress], hostname, wifi_mode, ip }`. `plug_state` is `unknown`, `off` or `on`, and is always a confirmed state. `state` is `idle`, `running`, `cooling`, `complete` or `error` |
| REQ-NET-008 | Pairing events broadcast over WebSocket: `pairing_open` (with countdown), `pairing_success`, `pairing_failed` |
| REQ-NET-009 | WebSocket control messages: `start`, `stop`, `start_pairing`, `unpair`, `plug_toggle`. A finished plug change is broadcast as `{type:"plug_result", ok, state, reason}`. A rejected `plug_toggle` is answered to the sender only, with `reason` `busy`, `unknown`, `not_paired`, `unavailable` or `running`. A refused `start` is answered with `{type:"run_result", ok:false, reason}` (`busy`, `updating`, `sensor`, `plug`, `profile`, `not_found`) |
| REQ-NET-010 | If the pre-connect scan finds zero networks, the STA bring-up is treated as failed and the device soft-restarts once per power cycle (RTC-flagged). The ESP32-C6 receiver intermittently comes up deaf after a cold boot; a chip-level restart clears it, a driver restart does not. Root cause unidentified |
| REQ-NET-011 | mDNS hostname is configurable and stored in NVS (`wifi_cfg/hostname`), default `reflow`, validated as a DNS label (a-z, 0-9, hyphen, max 32, no leading/trailing hyphen) |
| REQ-NET-012 | Further endpoints: `GET /api/wifi/scan`, `GET /api/wifi/config`, `POST /api/wifi/credentials` (accepts `ssid`/`password` and/or `hostname`, then reboots), `GET /api/run/trace` (the current or last run: elapsed, temperature, target, heater, power), `GET/POST /api/settings` (`temp_unit`), `GET /api/ota`, `POST /api/ota/check`, `POST /api/ota/install`, `POST /api/ota/upload` |
| REQ-NET-013 | Telemetry carries the name of the profile actually running (empty when idle) so the UI plots the correct target curve regardless of which client started the run |
| REQ-NET-014 | Broadcast enumerates httpd's own socket list and filters by `httpd_ws_get_fd_info()`. ESP-IDF 6.x does not invoke the URI handler for the WebSocket handshake, so a self-managed client list stays empty and telemetry reaches nobody |

## Web UI (REQ-WEB)
| Key        | Requirement |
|------------|-------------|
| REQ-WEB-001 | Four pages, embedded in the firmware: `index.html` (dashboard), `profiles.html` (profile editor), `network.html` (network settings), `settings.html` (temperature unit, firmware updates). Chart.js is bundled locally, so the pages need no internet access |
| REQ-WEB-002 | Responsive layout: single-column on mobile (< 768 px), two-column on large screens (≥ 768 px) |
| REQ-WEB-003 | Touch-friendly controls with minimum 44 px tap targets |
| REQ-WEB-004 | Live chart against real elapsed time: the profile (dashed) and the measured temperature; phase names in a framed row above the plot (the current one bold), split by dotted waypoint lines; a heater strip below it with grey blocks where the plug confirmed on and the requested power as a line. Nothing relies on colour alone. A page opened mid-run backfills from `/api/run/trace` |
| REQ-WEB-005 | Chart x-axis is time in seconds (grid every 30 s, 60 s on narrow charts), y-axis the temperature in the chosen unit; responsive and fills its container |
| REQ-WEB-006 | Profile selector dropdown, Start and Stop buttons via WebSocket, elapsed time, current phase label |
| REQ-WEB-007 | Zigbee plug section: status badge (Not paired / Unknown / Switching… / On / Off, confirmed state only), Pair Plug button (60 s countdown overlay), Unpair button |
| REQ-WEB-015 | Zigbee plug section has a **Test toggle** button. It is disabled unless the plug is available and its state is known with no change pending, and from the click until the device responds. The outcome is shown as a toast |
| REQ-WEB-008 | `profiles.html`: list, create, edit, rename, delete and upload profiles. Waypoint table with live SVG preview, and derived peak / total time / max ramp rate / time above 217 °C. Validates against firmware limits (16 waypoints, strictly increasing times, filename-safe names) |
| REQ-WEB-009 | `network.html`: current mode/address/SSID/hostname, network scan with channel and signal, SSID/password entry, hostname entry with live `.local` preview. Must remain usable in AP mode, so it uses no WebSocket |
| REQ-WEB-010 | Light and dark theme: follows the browser (`prefers-color-scheme`), with a theme button that overrides it per browser. Temperature unit °C (default) or °F, device-wide (Settings), converted only in the pages; firmware, profiles, console and logs stay in °C |
| REQ-WEB-011 | Sensor ambient temperature is displayed, amber from 65 °C and red with a warning from 80 °C (MLX90614ESF is rated to 85 °C ambient) |
| REQ-WEB-012 | Run faults are shown in the run state (for example "Stopped: sensor reading frozen while heating"). Plug-result popups appear only in answer to the Test toggle, or for a failure while no run is active; the heater strip shows the controller's own switching |
| REQ-WEB-013 | Zigbee controls are disabled with an explanation while in AP mode, where Zigbee cannot run; the profile selector is locked while a run is in progress and re-enabled when it completes |
| REQ-WEB-014 | No blocking `alert()`/`confirm()` dialogs: notifications are inline toasts and confirmations a modal, so they never stall the telemetry socket |
