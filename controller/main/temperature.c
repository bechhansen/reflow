#include "temperature.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_check.h"

#define MLX_ADDR     0x5A
#define MLX_REG_TA          0x06
#define MLX_REG_TOBJ 0x07
#define I2C_FREQ_HZ  100000
#define I2C_TIMEOUT  50

static const char *TAG = "temperature";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

esp_err_t temperature_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = CONFIG_REFLOW_I2C_SDA_PIN,
        .scl_io_num        = CONFIG_REFLOW_I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "I2C bus init failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MLX_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev), TAG, "MLX90614 add failed");

    ESP_LOGI(TAG, "MLX90614 ready on SDA=%d SCL=%d", CONFIG_REFLOW_I2C_SDA_PIN, CONFIG_REFLOW_I2C_SCL_PIN);
    return ESP_OK;
}

/* CRC-8 (SMBus, poly 0x07) over the full I2C transaction bytes */
static uint8_t smbus_crc8(uint8_t init, const uint8_t *data, size_t len)
{
    uint8_t crc = init;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}

/* Read one 16-bit MLX90614 RAM register over SMBus, PEC-verified.
   Shared by the object (Tobj1, 0x07) and ambient (Ta, 0x06) channels — both use
   the identical frame format and 0.02 K/LSB Kelvin scaling. */
static esp_err_t mlx_read_temp_reg(uint8_t reg, float *celsius)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;

    uint8_t buf[3]; /* data_lo, data_hi, pec */

    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, sizeof(buf), I2C_TIMEOUT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C read failed: %s", esp_err_to_name(err));
        i2c_master_bus_reset(s_bus);
        return err;
    }

    ESP_LOGD(TAG, "raw bytes: 0x%02x 0x%02x 0x%02x (pec)", buf[0], buf[1], buf[2]);

    /* Verify SMBus PEC over: [SLA_W, reg, SLA_R, data_lo, data_hi] */
    uint8_t pec_data[] = { MLX_ADDR << 1, reg, (MLX_ADDR << 1) | 1, buf[0], buf[1] };
    uint8_t expected   = smbus_crc8(0, pec_data, sizeof(pec_data));
    if (buf[2] != expected) {
        ESP_LOGW(TAG, "PEC mismatch: got 0x%02x expected 0x%02x — bad wiring?", buf[2], expected);
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if (raw & 0x8000) {
        ESP_LOGW(TAG, "MLX90614 error flag set (raw=0x%04x)", raw);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *celsius = (float)(raw & 0x7FFF) * 0.02f - 273.15f;
    ESP_LOGD(TAG, "reg 0x%02x: %.2f °C  (raw=0x%04x)", reg, (double)*celsius, raw);
    return ESP_OK;
}

esp_err_t temperature_read(float *celsius)
{
    return mlx_read_temp_reg(MLX_REG_TOBJ, celsius);
}

/* Ambient (die) temperature. This is the sensor's own body temperature, not the
   soleplate: the MLX90614ESF is only rated to +85 °C ambient, so it doubles as a
   warning that the bracket is letting the sensor cook. */
esp_err_t temperature_read_ambient(float *celsius)
{
    return mlx_read_temp_reg(MLX_REG_TA, celsius);
}
