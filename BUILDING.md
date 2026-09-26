# Building, Flashing and Releasing

## Prerequisites

- **ESP-IDF v6.1**: install it by following the [ESP-IDF getting started guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/get-started/).
- **An ESP32-C6 board with 8 MB flash**, connected over USB.
- **Python 3.9+**, installed along with ESP-IDF.

## 1. Build

```bash
. ~/esp/esp-idf-v6.1/export.sh      # or wherever ESP-IDF is installed
cd controller
idf.py set-target esp32c6           # once
idf.py build
```

The first build downloads the Espressif Zigbee libraries from the component
registry, so it needs internet access.

The web UI (`controller/web/`) and the default profiles (`controller/profiles/`)
are compressed and embedded in the firmware at build time, so there is no
separate filesystem image to build or flash.

Optional settings are under `idf.py menuconfig` → **Reflow Controller**:
- Wi-Fi SSID and password to build in (you can also set them from the
  `Reflow-Setup` access point)
- the I²C pins (SDA GPIO 6, SCL GPIO 7)
- the Zigbee endpoint
- the GitHub repository used for firmware updates

## 2. Flash over USB (first time, or recovery)

```bash
idf.py -p /dev/cu.usbmodemXXXX flash monitor     # Linux: /dev/ttyACM0
```

This writes the bootloader, the partition table, the OTA state and the
firmware. It never writes the profiles partition, so saved profiles survive a
reflash. Exit the monitor with **Ctrl-]**.

Without a build environment, flash a release's `reflow-<version>-usb.zip`
instead. Its `FLASH.txt` has the `esptool.py` command.

### Partition layout (8 MB)

| Partition | Offset | Size | Contents |
|---|---|---|---|
| nvs | 0x9000 | 24 K | Wi-Fi, tuning parameters, settings, plug pairing |
| phy_init | 0xF000 | 4 K | RF calibration |
| ota_0 | 0x10000 | 2 MB | firmware slot A |
| spiffs | 0x210000 | 512 K | your reflow profiles |
| zb_storage, zb_fct | 0x290000 | 20 K | Zigbee network |
| otadata | 0x295000 | 8 K | which slot boots |
| ota_1 | 0x300000 | 2 MB | firmware slot B |

## 3. First boot

1. **Join Wi-Fi:** with no Wi-Fi stored, the controller opens the access point **`Reflow-Setup`**, which
   is open with no password. Connect to it and open **http://192.168.4.1**.
2. **Choose your network:** under **Network settings**, pick your network, enter the password and
   save. The controller reboots and joins it.
3. **Open the UI:** at **http://reflow.local**, or at the IP shown in the serial log or your
   router.
4. **Pair the plug:** click **Pair**, then put the plug into pairing mode (usually by holding
   its button until the LED flashes). The badge shows **Off** once paired.

Zigbee runs only when connected to your Wi-Fi: the ESP32-C6's single radio
can't run Zigbee alongside its own access point.

## 4. Firmware updates over Wi-Fi

After the first USB flash, update from **Settings → Firmware**:

- **Install:** the controller checks GitHub for the latest **production release**
  60 s after boot, every 24 h, and when you press **Check now**. A newer one shows
  as **Install vX.Y.Z**, and also in the main page's header. It installs only when you click.
- **Upload firmware file:** installs a `.bin` from your computer, such as a local
  build (`controller/build/controller.bin`) or a pre-release's
  `reflow-controller.bin`.

**Safety:**
- Nothing installs while a run is active, and the heater is switched off first.
- The new firmware goes into the idle slot and must start properly and confirm itself.
  If it doesn't, the controller returns to the previous version by itself.
- Updates keep Wi-Fi, tuning, settings, profiles and the plug pairing.

## 5. Releasing

Releases are built by GitHub Actions (`.github/workflows/esp32-build.yml`)
from tags on `main`:

```bash
git tag v1.2.0
git push origin v1.2.0
```

| Tag | Result |
|---|---|
| `vX.Y.Z` | A production release. Controllers offer it as an update. |
| `vX.Y.Z-alpha…`, `-beta…`, `-rc…` | A pre-release, for manual upload only. Never offered to controllers. |
| any other push or pull request | Built and tested, kept as a 30-day workflow artifact. |

Each release has two assets:
- `reflow-controller.bin`: the firmware image, for updates and uploads.
- `reflow-<version>-usb.zip`: the first-time or recovery USB flash.

The firmware's version is the tag. CI writes it to `controller/version.txt`,
and local builds use `git describe`. If you change `version.txt` locally, run
`idf.py reconfigure`.

## 6. Tests

```bash
make -C controller/test/reflow_algo test    # control law against a simulated iron
make -C controller/test/plug_fsm test       # plug state machine
```

Both run in CI before every build.

## Tuning for your heat source

The built-in model describes the iron the project was developed on. Another
iron or hotplate heats and cools differently. Measure it once, with the heat
source on the plug and you **present**:

```bash
# The ESP-IDF Python environment has pyserial. Close idf.py monitor first.
PY=~/.espressif/python_env/idf6.1_py3.13_env/bin/python

# Full power to 200 °C, then cooling; about 5 minutes.
$PY controller/tools/hil.py step 1.0 300 200 --duration 300

# Fit the model: prints K, tau, theta, loss_quad, gains and the ctl set lines.
$PY controller/tools/hil.py fit controller/tools/logs/<step>.log --quad

# Apply and keep the values:
$PY controller/tools/hil.py cmd "ctl set K <value>"     # … one per printed line
$PY controller/tools/hil.py cmd "ctl save"
```

Then check it with a hold (`hil.py hold 150 --duration 240`) and a profile run
(`hil.py run "SMD291SNL SAC305"`), and analyze them with `hil.py analyze <log>`.
`ctl show` lists every parameter, and `ctl reset` returns to the defaults.

## Factory reset

```bash
idf.py -p /dev/cu.usbmodemXXXX erase-flash
idf.py -p /dev/cu.usbmodemXXXX flash
```

This erases Wi-Fi, tuning, settings, profiles and the plug pairing. The default
profiles are written again on the next boot.
