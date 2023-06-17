#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "iot_button.h"
#include "simple_step.hpp"

static const char *TAG_BUTTON_TMC2209_ENABLE = "Button TMC2209 Enable";

static QueueHandle_t button_isr_queue = NULL;
button_handle_t g_btns[1] = {0};
extern TaskHandle_t stepper_motor_handler;

// |================================================================================================ |
// |                                          Functions                                              |
// |================================================================================================ |

static void button_single_click_cb(void *arg, void *data_to_send)
{
    xQueueSendFromISR(button_isr_queue, (void *)&data_to_send, NULL);
}

static void setup_button(const gpio_pins_config_t *gpio_pins_config)
{
    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = gpio_pins_config->BUTTON_EN_PIN,
            .active_level = 0,
        },
    };
    g_btns[0] = iot_button_create(&cfg);

    // Create queue for button isr
    button_isr_queue = xQueueCreate(10, sizeof(gpio_num_t));

    // Register callback using button component
    iot_button_register_cb(g_btns[0], BUTTON_SINGLE_CLICK, button_single_click_cb, (void *)gpio_pins_config->BUTTON_EN_PIN);
}

// |================================================================================================ |
// |                                          Main Task                                              |
// |================================================================================================ |

static void button_tmc2209_enable_task(void *params)
{
    const gpio_pins_config_t *gpio_pins_config = (gpio_pins_config_t *)params;

    // Configure Button ISR Pin
    setup_button(gpio_pins_config);

    int data_received;
    int count = 0;
    while (true)
    {
        if (xQueueReceive(button_isr_queue, &data_received, portMAX_DELAY))
        {
            ESP_LOGI(TAG_BUTTON_TMC2209_ENABLE, "GPIO %d was pressed %d times. The state is %d\n", data_received, ++count, gpio_get_level(gpio_pins_config->BUTTON_EN_PIN));

            // Switch the driver on or off (0=ON, 1=OFF)
            int new_level = (gpio_get_level(gpio_pins_config->EN_PIN) + 1) % 2;
            ESP_LOGD(TAG_BUTTON_TMC2209_ENABLE, "new_level to set is: %d\n", new_level);
            gpio_set_level(gpio_pins_config->EN_PIN, new_level);
            
            switch (new_level)
            {
            case 0: // Enable the driver
                vTaskResume(stepper_motor_handler);
                
                break;
            case 1: // Disable the driver
                vTaskSuspend(stepper_motor_handler);
                break;
            }
            ESP_LOGI(TAG_BUTTON_TMC2209_ENABLE, "EN_PIN level is: %d\n", gpio_get_level(gpio_pins_config->EN_PIN));
        }
    }

    iot_button_delete(g_btns[0]);
}


#ifdef __cplusplus
}
#endif