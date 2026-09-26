# Additional permission under GNU GPL version 3 section 7

This program is licensed under the GNU General Public License version 3
(see [LICENSE](LICENSE)), with the following additional permission.

The firmware runs on Espressif ESP32 chips and must be linked with libraries
that Espressif distributes only in binary form:
- **ESP-IDF's binary-only libraries**, for example the Wi-Fi, PHY/RF, coexistence and
  IEEE 802.15.4 libraries.
- **The Espressif Zigbee SDK**: `esp-zigbee-lib`, and `esp-zboss-lib` with the ZBOSS
  stack by DSR Corporation.

Their source code is not available, so without this permission the
GPL's Corresponding Source requirement could not be met by anyone.

> If you modify this Program, or any covered work, by linking or combining it
> with the binary-only libraries that Espressif Systems distributes as part of
> ESP-IDF or the Espressif Zigbee SDK (esp-zigbee-lib and esp-zboss-lib,
> including the ZBOSS stack of DSR Corporation), or modified versions of those
> libraries, containing parts covered by the terms of their own licenses, the
> licensors of this Program grant you additional permission to convey the
> resulting work. Corresponding Source for a non-source form of such a
> combination need not include the source code of the parts of those libraries
> that Espressif distributes only in binary form, but must include the
> Corresponding Source of this Program and of every other component used that
> is available in source form.

**What this changes:** only the binary-only Espressif components. Everything
else in this project, and every derivative of it, stays under GPL v3. You may
remove this permission from your own modified versions (GPL v3 section 7).
