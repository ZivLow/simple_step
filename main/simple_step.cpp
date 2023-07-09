#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "simple_step.hpp"
#include "tasks/button_tmc2209_enable_task.hpp"
#include "tasks/rotary_sensor_task.hpp"
#include "tasks/stepper_motor_task.hpp"

TaskHandle_t button_tmc2209_enable_handler;
TaskHandle_t rotary_sensor_handler;
TaskHandle_t stepper_motor_handler;



void app_main()
{
    

    static gpio_pins_config_t gpio_pins_config = {
        .UART_PORT_NUM  = UART_NUM_2,
        .DRIVER_ADDRESS = 0b00,         // TMC2209 Driver address according to MS1 and MS2
        .EN_PIN         = GPIO_NUM_23,  // Enable
        .DIR_PIN        = GPIO_NUM_18,  // Direction
        .STEP_PIN       = GPIO_NUM_19,  // Step
        .CS_PIN         = GPIO_NUM_NC,  // Chip select
        .SW_MOSI        = GPIO_NUM_NC,  // Software Master Out Slave In (MOSI)
        .SW_MISO        = GPIO_NUM_NC,  // Software Master In Slave Out (MISO)
        .SW_SCK         = GPIO_NUM_NC,  // Software Slave Clock (SCK)
        .SW_RX          = GPIO_NUM_16,  // TMC2208/TMC2224 SoftwareSerial receive pin
        .SW_TX          = GPIO_NUM_17,  // TMC2208/TMC2224 SoftwareSerial transmit pin
        .BUTTON_EN_PIN  = GPIO_NUM_4,   // Button to enable or disable the motor driver
    };

    xTaskCreatePinnedToCore(&button_tmc2209_enable_task, "Button TMC2209 Enable Task", 4096, &gpio_pins_config, 4, &button_tmc2209_enable_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&rotary_sensor_task, "Rotary Sensor Task", 4096, NULL, 3, &rotary_sensor_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&stepper_motor_task, "Stepper Motor Task", 4096, &gpio_pins_config, 3, &stepper_motor_handler, tskNO_AFFINITY);

    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
