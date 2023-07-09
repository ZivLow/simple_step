#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "simple_step.hpp"
#include "tasks/stepper_motor_task.hpp"

#define UART_MAX_DELAY    100
#define CRC_TABLE_INDEX   1             // Choose CRC table index 0 or 1

static const char *TAG = "TMC2209 Driver";

static TaskHandle_t stepper_periodic_handler;

// Declare struct instances for TMC2209 driver and its condiguration
TMC2209TypeDef tmc2209_driver_def;
TMC2209TypeDef *tmc2209_driver = &tmc2209_driver_def;
ConfigurationTypeDef tmc_2209_driver_config_def;
ConfigurationTypeDef *tmc_2209_driver_config = &tmc_2209_driver_config_def;

// Struct for TMC ramp
TMC_LinearRamp tmc_ramp_def;
TMC_LinearRamp *tmc_ramp = &tmc_ramp_def;

// |================================================================================================ |
// |                                          Functions                                              |
// |================================================================================================ |


void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength)
{
    uart_write_bytes(channel, data, writeLength);
    uart_wait_tx_done(channel, UART_MAX_DELAY);
    uart_flush(channel);
    if (readLength){
        uart_read_bytes(channel, data, readLength, UART_MAX_DELAY);
    }
}

// CRC8 function from TMC-API. Index is 0 for channel 1, and 1 for channel 2.
uint8_t tmc2209_CRC8(uint8_t *datagram, size_t datagramLength)
{
    return tmc_CRC8(datagram, datagramLength, CRC_TABLE_INDEX);
}

static void configure_gpio(const gpio_pins_config_t *gpio_pins_config)
{
    // Configure GPIO for TMC2209 driver
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;                            //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                                  //set as output mode
    io_conf.pin_bit_mask = ((1ULL<<gpio_pins_config->EN_PIN) | (1ULL<<gpio_pins_config->STEP_PIN) | (1ULL<<gpio_pins_config->DIR_PIN));        //bit mask of the pins that you want to set,e.g.GPIO18
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                     //disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                         //disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf));                          //configure GPIO with the given settings

    // Set to input and output mode, so can control enable pin voltage, and read its voltage.
    gpio_set_direction(gpio_pins_config->EN_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(gpio_pins_config->EN_PIN, 1);         // Disable driver in hardware
}

static void configure_uart(uint32_t baudrate, const gpio_pins_config_t *gpio_pins_config)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = static_cast<int>(baudrate),
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;       // Disable UART interrupt

    ESP_ERROR_CHECK(uart_driver_install(gpio_pins_config->UART_PORT_NUM, UART_FIFO_LEN * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(gpio_pins_config->UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(gpio_pins_config->UART_PORT_NUM, gpio_pins_config->SW_TX, gpio_pins_config->SW_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush(gpio_pins_config->UART_PORT_NUM);
}

// tmc2209_periodicJob function needs to be called periodically to update the registers
static void stepper_motor_periodic_task(void *params)
{
    while (true) {
        tmc2209_periodicJob(tmc2209_driver, 50);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// |================================================================================================ |
// |                                          Main Task                                              |
// |================================================================================================ |

void stepper_motor_task(void *params)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    const gpio_pins_config_t *gpio_pins_config = (gpio_pins_config_t *)params;

    // TMC Ramp Setup
    tmc_ramp_linear_init(tmc_ramp);
    tmc_ramp_linear_set_precision(tmc_ramp, 5);
    tmc_ramp_linear_set_maxVelocity(tmc_ramp, 6000);
    tmc_ramp_linear_set_acceleration(tmc_ramp, 150);

    // For position type control
    tmc_ramp_linear_set_mode(tmc_ramp, TMC_RAMP_LINEAR_MODE_POSITION);
    tmc_ramp_linear_set_targetPosition(tmc_ramp, 180);

    // For velocity type control
    // tmc_ramp_linear_set_mode(tmc_ramp, TMC_RAMP_LINEAR_MODE_VELOCITY);
    // tmc_ramp_linear_set_targetVelocity(tmc_ramp, 312);
    

    // Setup gpio and uart for TMC2209 driver
    configure_gpio(gpio_pins_config);
    configure_uart(115200, gpio_pins_config);

    // Generate CRC 8-bit table for polynomial x^8 + x^2 + x + 1
    // Need to reverse table because data is flipped.
    // Index 0 for channel 1. Index 1 is for channel 2.
    tmc_fillCRC8Table((uint8_t)0b100000111, true, CRC_TABLE_INDEX);

    // Initialize TMC2209 driver
    tmc2209_init(tmc2209_driver, gpio_pins_config->UART_PORT_NUM, gpio_pins_config->DRIVER_ADDRESS, tmc_2209_driver_config, tmc2209_defaultRegisterResetState);

    // Create periodic task to update TMC2209 registers
    xTaskCreatePinnedToCore(stepper_motor_periodic_task, "stepper_motor_periodic_task", 4096, NULL, 3, &stepper_periodic_handler, tskNO_AFFINITY);
    tmc2209_reset(tmc2209_driver);

    // Need to disable PDN for UART read
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT, 1);
    // Set toff
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_CHOPCONF, TMC2209_TOFF_MASK, TMC2209_TOFF_SHIFT, 5);
    // Set microstepping
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_CHOPCONF, TMC2209_MRES_MASK, TMC2209_MRES_SHIFT, 0);
    // Set blank time
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_CHOPCONF, TMC2209_TBL_MASK, TMC2209_TBL_SHIFT, 0);
    // Set hold current
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_MASK, TMC2209_IHOLD_SHIFT, 0);
    // Set run current
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_IHOLD_IRUN, TMC2209_IRUN_MASK, TMC2209_IRUN_SHIFT, 1);
    // Set hold current decay delay
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_IHOLD_IRUN, TMC2209_IHOLDDELAY_MASK, TMC2209_IHOLDDELAY_SHIFT, 15);
    // Set freewheeling
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_PWMCONF, TMC2209_FREEWHEEL_MASK, TMC2209_FREEWHEEL_SHIFT, 1);
    // StealthChop --> SpreadCycle threshold velocity
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_TPWMTHRS, TMC2209_TPWMTHRS_MASK, TMC2209_TPWMTHRS_SHIFT, 500);
    // Set StealthChop (StealthChop ignores current settings)
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_PWMCONF, TMC2209_PWM_AUTOSCALE_MASK, TMC2209_PWM_AUTOSCALE_SHIFT, 1);
    TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_PWMCONF, TMC2209_PWM_GRAD_MASK, TMC2209_PWM_GRAD_SHIFT, 1);

    // while (tmc2209_driver->config->state != CONFIG_READY) {
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }
    gpio_set_level(gpio_pins_config->EN_PIN, 0);         // Enable driver in hardware


    // static bool direction = false;

    int32_t data_received;

    if (xQueueReceive(motor_angle_queue, &data_received, portMAX_DELAY))
    {
        ESP_LOGD(TAG, "Receiving from motor_angle_queue: %" PRIi32, (int32_t) data_received);
        // tmc_ramp_linear_set_rampPosition(tmc_ramp, (int32_t) data_received);
        tmc_ramp_linear_set_rampPosition(tmc_ramp, 0);
    }

    

    while (true) {
        // // Change direction
        // direction = !direction;
        // TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_GCONF, TMC2209_SHAFT_MASK, TMC2209_SHAFT_SHIFT, direction);

        // // Set motor velocity
        // TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, 5000);
        
        // int32_t gconf_status = TMC2209_FIELD_READ(tmc2209_driver, TMC2209_GCONF, TMC2209_PDN_DISABLE_MASK, TMC2209_PDN_DISABLE_SHIFT);
        // ESP_LOGD(TAG, "gconf_status: %" PRIi32, gconf_status);

        // int32_t driver_version_status = TMC2209_FIELD_READ(tmc2209_driver, TMC2209_IOIN, TMC2209_VERSION_MASK, TMC2209_VERSION_SHIFT);
        // ESP_LOGD(TAG, "driver_version_status: %" PRIi32, driver_version_status);
        
        // vTaskDelay(3000 / portTICK_PERIOD_MS);

        

        if (xQueueReceive(motor_angle_queue, &data_received, UART_MAX_DELAY))
        {
            ESP_LOGD(TAG, "Receiving from motor_angle_queue: %" PRIi32, (int32_t) data_received);
            // tmc_ramp_linear_set_rampPosition(tmc_ramp, (int32_t) data_received);
        }

        int32_t computed_velocity = tmc_ramp_linear_compute(tmc_ramp);
        ESP_LOGD(TAG, "LinearRamp State: %" PRIu32, (uint32_t) tmc_ramp_linear_get_state(tmc_ramp));
        ESP_LOGD(TAG, "Readout from linearRamp: %" PRIu32, (uint32_t) tmc_ramp_linear_get_acceleration_limit(tmc_ramp));
        ESP_LOGD(TAG, "New computed velocity: %" PRIi32, computed_velocity);

        // Set motor velocity
        TMC2209_FIELD_WRITE(tmc2209_driver, TMC2209_VACTUAL, TMC2209_VACTUAL_MASK, TMC2209_VACTUAL_SHIFT, computed_velocity);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
}
