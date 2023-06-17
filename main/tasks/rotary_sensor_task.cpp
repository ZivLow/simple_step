#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver_as5600_basic.h"

static const char *TAG_ROTARY_SENSOR = "Rotary_Sensor";
as5600_handle_t gs_handle; /**< as5600 handle */

// |================================================================================================ |
// |                                          Main Task                                              |
// |================================================================================================ |

static void rotary_sensor_task(void *params)
{
    as5600_info_t info;

    uint8_t AS5600_status;
    uint8_t AS5600_agc;
    uint16_t AS5600_magnitude;
    float deg; // degree of the stepper motor

    uint16_t start_pos; // start position of AS5600
    uint16_t stop_pos;  // stop position of AS5600
    uint16_t max_angle; // max angle allowed on AS5600

    static uint16_t set_start_pos = 2;
    // static uint16_t set_stop_pos = 2000;
    static uint16_t set_max_angle = 280;

    
    // Initialize AS5600 magnetic rotary sensor
    ESP_ERROR_CHECK(as5600_basic_init());

    /* get chip information */
    ESP_ERROR_CHECK(as5600_info(&info));
    /* print AS5600 chip information */
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: chip is %s.\n", info.chip_name);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: manufacturer is %s.\n", info.manufacturer_name);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: interface is %s.\n", info.interface);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: max current is %0.2fmA.\n", info.max_current_ma);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: max temperature is %0.1fC.\n", info.temperature_max);
    ESP_LOGI(TAG_ROTARY_SENSOR, "as5600: min temperature is %0.1fC.\n", info.temperature_min);

    // Set the range of the AS5600 magnetic rotary sensor. 
    // Choose (start position + stop position) setup, or (start position + max angle) setup.
    ESP_ERROR_CHECK(as5600_set_start_position(&gs_handle, set_start_pos));
    // ESP_ERROR_CHECK(as5600_set_stop_position(&gs_handle, set_stop_pos));
    as5600_set_max_angle(&gs_handle, set_max_angle);

    while (true)
    {
        // Get AS5600 chip status
        as5600_get_status(&gs_handle, &AS5600_status);

        if (AS5600_status & AS5600_STATUS_MD)
        {
            ESP_LOGI(TAG_ROTARY_SENSOR, "Magnet detected.\n");

            if (AS5600_status & AS5600_STATUS_MH)
            {
                ESP_LOGW(TAG_ROTARY_SENSOR, "Magnet is too strong.\n");
            }
            else if (AS5600_status & AS5600_STATUS_ML)
            {
                ESP_LOGW(TAG_ROTARY_SENSOR, "Magnet is too weak.\n");
            }
            else
            {
                ESP_LOGD(TAG_ROTARY_SENSOR, "Magnet is just right :)\n");
            }
        }
        else
        {
            ESP_LOGW(TAG_ROTARY_SENSOR, "Magnet not found!.\n");
        }

        // Debug for AS5600 status
        ESP_LOGD(TAG_ROTARY_SENSOR, "AS5600_status = %d.\n", AS5600_status);

        // Get the gain of the Automatic Gain Control (AGC). A middle value is preferred. 5V: 0-255. 3.3V: 0-128.
        as5600_get_agc(&gs_handle, &AS5600_agc);

        // Debug for AS5600 AGC Gain value
        ESP_LOGD(TAG_ROTARY_SENSOR, "AS5600_agc = %d.\n", AS5600_agc);

        // Get the magnitude of the magnetic field
        as5600_get_magnitude(&gs_handle, &AS5600_magnitude);

        // Debug for AS5600 magnetic field magnitude. 0-4095
        ESP_LOGD(TAG_ROTARY_SENSOR, "AS5600_magnitude = %d.\n", AS5600_magnitude);

        // Read stepper angle from AS5600
        ESP_ERROR_CHECK(as5600_basic_read(&deg));
        ESP_LOGI(TAG_ROTARY_SENSOR, "Stepper angle is: %.2f degrees.\n", deg);

        as5600_get_start_position(&gs_handle, &start_pos);
        ESP_LOGI(TAG_ROTARY_SENSOR, "AS5600 start position is: %d.\n", start_pos);

        as5600_get_stop_position(&gs_handle, &stop_pos);
        ESP_LOGI(TAG_ROTARY_SENSOR, "AS5600 stop position is: %d.\n", stop_pos);

        as5600_get_max_angle(&gs_handle, &max_angle);
        ESP_LOGI(TAG_ROTARY_SENSOR, "AS5600 max angle is: %d degrees.\n", max_angle);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    as5600_basic_deinit();
}

#ifdef __cplusplus
}
#endif