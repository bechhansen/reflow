# Building and Flashing the Reflow Controller

## Prerequisites

- **ESP-IDF v5.5 or later** — install from [https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/)
- ESP32-C6 development board connected via USB
- Python 3.9+ (installed by the IDF toolchain setup)

---

## 1. Set Up the Environment

Open a terminal and activate the ESP-IDF environment:

```bash
source ~/esp/esp-idf/export.sh
```

Verify the target is set to ESP32-C6:

```bash
cd controller
idf.py get-target
# Should print: esp32c6
```

If not already set:

```bash
idf.py set-target esp32c6
```

---

## 2. Configure Wi-Fi Credentials (optional)

Bake default Wi-Fi credentials into the build. This is optional — you can also configure Wi-Fi via the `Reflow-Setup` AP after first boot.

```bash
idf.py menuconfig
```

Navigate to **Component config → Reflow Controller** and set:
- `Default Wi-Fi SSID` — your network name
- `Default Wi-Fi Password` — your password

Other configurable items in the same menu:
- `I2C SDA pin` (default: GPIO 6)
- `I2C SCL pin` (default: GPIO 7)
- `Zigbee local endpoint number` (default: 10)

---

## 3. Build the Firmware

The first build downloads the Zigbee SDK managed components (`espressif/esp-zboss-lib` and `espressif/esp-zigbee-lib`) from the component registry — requires internet access.

```bash
idf.py build
```

A successful build ends with:

```
Generated /path/to/controller/build/controller.bin
```

The SPIFFS image (`spiffs.bin`) containing the web UI and default profiles is also generated automatically.

---

## 4. Flash Firmware

Find your device port (typically `/dev/tty.usbmodem*` on macOS or `/dev/ttyUSB0` on Linux):

```bash
ls /dev/tty.usb*     # macOS
ls /dev/ttyUSB*      # Linux
```

Flash only the firmware (fastest for iterating on code changes):

```bash
idf.py -p /dev/tty.usbmodemXXXX flash
```

---

## 5. Flash the SPIFFS Filesystem

The SPIFFS partition holds the web UI (`/spiffs/www/index.html`) and the default reflow profiles (`/spiffs/profiles/*.json`).

**Flash SPIFFS only** (use this when updating web UI or profiles without reflashing firmware):

```bash
idf.py -p /dev/tty.usbmodemXXXX spiffs-flash
```

**Flash everything at once** (firmware + bootloader + partition table + SPIFFS):

```bash
idf.py -p /dev/tty.usbmodemXXXX flash
```

> The top-level `CMakeLists.txt` uses `spiffs_create_partition_image(spiffs spiffs_image FLASH_IN_PROJECT)`, so `idf.py flash` automatically includes the SPIFFS image.

---

## 6. Monitor Serial Output

```bash
idf.py -p /dev/tty.usbmodemXXXX monitor
```

On a successful boot you should see:

```
I (...)  wifi_mgr: STA connected, IP: 192.168.x.x
I (...)  web_server: HTTP server started
I (...)  zigbee_plug: Zigbee coordinator started (factory-new)
```

Exit the monitor with **Ctrl-]**.

---

## 7. Flash + Monitor in One Step

```bash
idf.py -p /dev/tty.usbmodemXXXX flash monitor
```

---

## 8. Erase Flash (Factory Reset)

Clears all NVS data (Wi-Fi credentials, Zigbee pairing, uploaded profiles):

```bash
idf.py -p /dev/tty.usbmodemXXXX erase-flash
```

After erasing, reflash everything:

```bash
idf.py -p /dev/tty.usbmodemXXXX flash
```

---

## 9. First Boot Workflow

1. Power on the ESP32-C6.
2. If no Wi-Fi credentials are stored, the device starts as the **`Reflow-Setup`** access point (no password).
3. Connect your phone or laptop to `Reflow-Setup`.
4. Open **http://192.168.4.1** in a browser.
5. Go to the **Wi-Fi Settings** tab, scan for your network, enter the password, and save. The device reboots and connects to your network.
6. Find the device's new IP address in the serial monitor output, or from your router's DHCP table.
7. Open the web UI at `http://<device-ip>`.

---

## 10. Pairing a Zigbee Smart Plug

1. In the web UI, go to the **Zigbee Plug** section.
2. Click **Pair Plug**. A 60-second countdown starts.
3. Put your smart plug into pairing mode (usually by holding the button until the LED flashes).
4. The controller will detect the plug, store its address, and show **Paired** status.

---

## Updating Web UI or Profiles Without Full Reflash

Edit files in `controller/spiffs_image/www/` or `controller/spiffs_image/profiles/`, then:

```bash
idf.py build            # regenerates spiffs.bin
idf.py -p /dev/tty.usbmodemXXXX spiffs-flash
```

No firmware reflash needed.

---

## Partition Layout (4 MB flash)

| Partition  | Type | Size   | Contents                      |
|------------|------|--------|-------------------------------|
| nvs        | data | 24 KB  | Wi-Fi credentials, Zigbee pairing |
| phy_init   | data | 4 KB   | RF calibration data           |
| factory    | app  | 1.5 MB | Firmware                      |
| spiffs     | data | 512 KB | Web UI + reflow profiles       |
