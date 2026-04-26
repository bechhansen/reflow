# Reflow Hotplate — Requirements

## Hardware (REQ-HW)
| Key        | Requirement |
|------------|-------------|
| REQ-HW-001 | MCU is ESP32-C6 (native 802.15.4 Zigbee + Wi-Fi) |
| REQ-HW-002 | Temperature sensor is MLX90614ESF (IR contactless, I2C/SMBus, 3.3 V, address 0x5A) |
| REQ-HW-003 | Heat source is an inverted clothes iron (soleplate up as hotplate) |
| REQ-HW-004 | Heat is controlled by toggling a Zigbee smart wall plug (binary on/off, mechanical relay) |
| REQ-HW-005 | Relay must not switch more than once every 2 seconds |
| REQ-HW-006 | 3D-printed enclosure for the ESP32-C6 controller board |
| REQ-HW-007 | 3D-printed stand/cradle for the inverted iron |
| REQ-HW-008 | 3D-printed bracket positioning the MLX90614 above the soleplate |

## Firmware (REQ-FW)
| Key        | Requirement |
|------------|-------------|
| REQ-FW-001 | Firmware built with ESP-IDF targeting esp32c6 |
| REQ-FW-002 | MLX90614 driver reads object temperature via I2C at 100 kHz; resolution 0.02 °C |
| REQ-FW-003 | Zigbee stack runs as coordinator; sends On/Off commands (cluster 0x0006) to paired plug |
| REQ-FW-004 | Zigbee pairing opens a 60-second permit-join window; paired device short address stored in NVS |
| REQ-FW-005 | Unpair removes the NVS entry; controller blocks reflow start if no plug is paired |
| REQ-FW-006 | Control algorithm is time-proportional PID with 8-second cycle and 2-second minimum ON/OFF time (5 discrete duty levels: 0 25 50 75 100 %) |
| REQ-FW-007 | During ramp segments > 10 °C from setpoint: force 100 % duty; hand off to PID within 10 °C |
| REQ-FW-008 | Cooling segment forces 0 % duty unconditionally |
| REQ-FW-009 | PID uses derivative-on-measurement and integral anti-windup |
| REQ-FW-010 | Reflow profiles stored as JSON in SPIFFS (`/spiffs/profiles/`) |
| REQ-FW-011 | Default profiles shipped in SPIFFS: SAC305 Lead-Free and Sn63Pb37 Leaded |
| REQ-FW-012 | Profile format uses time-temperature waypoints; controller interpolates linearly between them |
| REQ-FW-013 | Web UI static files served from SPIFFS (`/spiffs/www/`) |
| REQ-FW-014 | UART console: logging output with IP address, profile count, and error messages at boot |

## Networking (REQ-NET)
| Key        | Requirement |
|------------|-------------|
| REQ-NET-001 | On boot, device attempts Wi-Fi STA connection using NVS-stored credentials |
| REQ-NET-002 | If no credentials exist or STA connection fails within 10 s, device starts AP mode (SSID: `Reflow-Setup`) |
| REQ-NET-003 | If STA connection drops, device retries 3× then falls back to AP mode |
| REQ-NET-004 | HTTP server provides REST API for profile CRUD: `GET/POST/DELETE /api/profiles/{name}` |
| REQ-NET-005 | WebSocket endpoint at `ws://device/ws` handles all real-time control and telemetry |
| REQ-NET-006 | WebSocket supports up to 4 simultaneous clients; messages broadcast to all connected clients |
| REQ-NET-007 | Telemetry broadcast at 2 Hz: `{ type, temp, setpoint, elapsed, phase, state, plug_paired, plug_on, duty_steps, wifi_mode, ip }` |
| REQ-NET-008 | Pairing events broadcast over WebSocket: `pairing_open` (with countdown), `pairing_success`, `pairing_failed` |
| REQ-NET-009 | WebSocket control messages: `start`, `stop`, `start_pairing`, `unpair` |

## Web UI (REQ-WEB)
| Key        | Requirement |
|------------|-------------|
| REQ-WEB-001 | Single self-contained `index.html` (HTML + CSS + JS) served from SPIFFS |
| REQ-WEB-002 | Responsive layout: single-column on mobile (< 768 px), two-column on large screens (≥ 768 px) |
| REQ-WEB-003 | Touch-friendly controls with minimum 44 px tap targets |
| REQ-WEB-004 | Live temperature graph using Chart.js: actual temperature (streaming) + profile target curve |
| REQ-WEB-005 | Graph X-axis = elapsed seconds, Y-axis = °C; responsive and fills its container |
| REQ-WEB-006 | Profile selector dropdown, Start and Stop buttons via WebSocket, elapsed time, current phase label |
| REQ-WEB-007 | Zigbee plug section: paired/on/off status badge, Pair Plug button (60 s countdown overlay), Unpair button |
| REQ-WEB-008 | Profile management: upload via file picker, list existing profiles, delete profile |
| REQ-WEB-009 | Wi-Fi settings page: scan networks, enter SSID/password, save credentials to NVS and reboot |
