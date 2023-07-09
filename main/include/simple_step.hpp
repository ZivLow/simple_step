#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/gpio.h"
#include "driver/uart.h"

extern QueueHandle_t motor_angle_queue;

typedef struct gpio_pins_config_s {
        uart_port_t UART_PORT_NUM;
        uint8_t DRIVER_ADDRESS; // TMC2209 Driver address according to MS1 and MS2

        const gpio_num_t
            EN_PIN     , // Enable
            DIR_PIN    , // Direction
            STEP_PIN   , // Step
            CS_PIN     , // Chip select
            SW_MOSI    , // Software Master Out Slave In (MOSI)
            SW_MISO    , // Software Master In Slave Out (MISO)
            SW_SCK     , // Software Slave Clock (SCK)
            SW_RX      , // TMC2208/TMC2224 SoftwareSerial receive pin
            SW_TX      , // TMC2208/TMC2224 SoftwareSerial transmit pin

            BUTTON_EN_PIN; // Button to enable or disable the motor driver
    } gpio_pins_config_t;

// |================================================================================================ |
// |                                          Functions                                              |
// |================================================================================================ |

void app_main();

#ifdef __cplusplus
}
#endif