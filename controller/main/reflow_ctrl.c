#include "reflow_ctrl.h"
#include "temperature.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ── Sensor shell ────────────────────────────────────────────────
   The reflow control law has been removed. What is left samples the MLX90614
   so temperature telemetry keeps flowing, and holds the run state the web UI
   reads. Nothing here drives the relay.

   reflow_ctrl_start() and reflow_ctrl_stop() are intentionally empty: the
   Start and Stop buttons still exist in the UI, but command nothing. */

/* The MLX90614 updates well below 10 Hz, so polling faster than the 2 Hz
   telemetry broadcast would mostly re-read the same value. */
#define SENSOR_POLL_MS  500

static const char *TAG = "reflow_ctrl";

static SemaphoreHandle_t s_mutex;
static ctrl_state_t      s_state     = CTRL_STATE_IDLE;
static float             s_temp      = 0.0f;
static float             s_ambient   = 0.0f;
static bool              s_sensor_ok = false;

static void sensor_task(void *arg)
{
    while (1) {
        float probe_temp, probe_amb;
        bool ok     = (temperature_read(&probe_temp) == ESP_OK);
        bool amb_ok = (temperature_read_ambient(&probe_amb) == ESP_OK);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (amb_ok) s_ambient = probe_amb;
        if (ok != s_sensor_ok)
            ESP_LOGI(TAG, "Sensor %s", ok ? "connected" : "disconnected");
        s_sensor_ok = ok;
        if (ok) s_temp = probe_temp;
        xSemaphoreGive(s_mutex);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
}

esp_err_t reflow_ctrl_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    xTaskCreate(sensor_task, "sensor", 3072, NULL, 6, NULL);
    return ESP_OK;
}

esp_err_t reflow_ctrl_start(const reflow_profile_t *profile)
{
    (void)profile;
    return ESP_OK;
}

esp_err_t reflow_ctrl_stop(void)
{
    return ESP_OK;
}

void reflow_ctrl_get_status(ctrl_status_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    out->temp       = s_temp;
    out->ambient    = s_ambient;
    out->sensor_ok  = s_sensor_ok;
    out->state      = s_state;
    xSemaphoreGive(s_mutex);
}
