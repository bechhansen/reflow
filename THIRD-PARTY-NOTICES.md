# Third-party notices

This project, and the firmware built from it, includes or links the following
third-party components. Each is used under its own license. Keep this file with
any redistribution of the firmware.

| Component | Where | License |
|---|---|---|
| [Chart.js](https://www.chartjs.org) v4 | `controller/web/chart.js`, embedded in the firmware | MIT, © Chart.js Contributors |
| [ESP-IDF](https://github.com/espressif/esp-idf) v6.1 | Linked into the firmware; fetched from Espressif | Apache 2.0, © Espressif Systems. Includes third-party parts under their own permissive licenses (FreeRTOS: MIT; lwIP, wpa_supplicant: BSD; mbedTLS: Apache 2.0; newlib: BSD-style; and others). See ESP-IDF's [copyrights and licenses](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/COPYRIGHT.html). The Wi-Fi, PHY, coexistence and 802.15.4 libraries are distributed in binary form only. |
| [esp-zigbee-lib](https://components.espressif.com/components/espressif/esp-zigbee-lib) | Linked into the firmware; fetched at build time | Apache 2.0, © Espressif Systems; binary form only |
| [esp-zboss-lib](https://components.espressif.com/components/espressif/esp-zboss-lib) (ZBOSS) | Linked into the firmware; fetched at build time | BSD-style, © DSR Corporation and Espressif Systems; binary form only. Redistribution in binary form is permitted as embedded in an Espressif chip product or a software update for it; it may not be reverse engineered, decompiled, modified or disassembled. |
| [cJSON](https://github.com/DaveGamble/cJSON) | Linked into the firmware; fetched at build time | MIT, © Dave Gamble and cJSON contributors |
| [mdns](https://components.espressif.com/components/espressif/mdns) | Linked into the firmware; fetched at build time | Apache 2.0, © Espressif Systems |

The binary-only Espressif components are combined with this GPL v3 program
under the additional permission in [LICENSE-EXCEPTION.md](LICENSE-EXCEPTION.md).

The full license texts come with each component. For the components fetched
at build time, they are in `controller/managed_components/*/LICENSE` after a
build, and in the ESP-IDF source tree.

---

### MIT License (Chart.js; cJSON)

> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.

### Apache License 2.0 (ESP-IDF, esp-zigbee-lib, mdns)

The full text is at https://www.apache.org/licenses/LICENSE-2.0 and in each
component's `LICENSE` file.
