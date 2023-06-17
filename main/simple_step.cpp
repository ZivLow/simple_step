

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "simple_step.hpp"
#include "button_tmc2209_enable_task.cpp"
#include "rotary_sensor_task.cpp"
#include "stepper_motor_task.cpp"


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





    // // Configuring ISR
    // gpio_config_t io_isr_conf;
    // io_isr_conf.intr_type = GPIO_INTR_NEGEDGE;                            //disable interrupt
    // io_isr_conf.mode = GPIO_MODE_INPUT;                                   //set as output mode
    // io_isr_conf.pin_bit_mask = (1ULL<<BUTTON_PIN);                        //bit mask of the pins that you want to set,e.g.GPIO18
    // io_isr_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                     //disable pull-down mode
    // io_isr_conf.pull_up_en = GPIO_PULLUP_ENABLE;                          //disable pull-up mode
    // ESP_ERROR_CHECK(gpio_config(&io_isr_conf));                           //configure GPIO with the given settings

    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, (void*) BUTTON_PIN);