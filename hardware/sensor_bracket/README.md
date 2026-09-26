# Sensor Bracket

Positions the MLX90614ESF IR thermometer above the iron soleplate.

## Purpose
Holds the MLX90614 at the correct height and angle to measure the soleplate temperature without contact, aimed squarely at the hotplate surface.

## Sensor Specs (MLX90614ESF)
- Supply: 3.3 V (direct connection to ESP32-C6)
- Interface: I2C / SMBus, address `0x5A`
- Object temperature range: −70 °C to +382 °C
- Field of view depends on the variant: 90° for the common MLX90614ESF-BAA (GY-906
  modules), about 5° for the -DCI. With 90°, keep the sensor close (below) so it
  sees mostly soleplate; aim it perpendicular to the plate
- Recommended measurement distance: 5–20 mm above soleplate

## Print Settings
| Parameter | Value |
|-----------|-------|
| Material  | PETG minimum; ABS preferred (sensor bracket is near the hot surface) |
| Layer height | 0.15 mm |
| Infill | 30% |

## Design Notes
- Mount the sensor 10–15 mm above the soleplate surface, aimed perpendicular
- Route the I2C cable (4-wire: VCC, GND, SDA, SCL) away from the soleplate edge
- Use a short cable run to the controller enclosure to reduce I2C noise

## Wiring
| MLX90614 Pin | ESP32-C6 |
|---|---|
| VDD | 3.3 V |
| VSS | GND |
| SDA | GPIO 6 (default, configurable via menuconfig) |
| SCL | GPIO 7 (default, configurable via menuconfig) |

Note: 4.7 kΩ pull-up resistors are required on SDA and SCL if not already present on the development board.
