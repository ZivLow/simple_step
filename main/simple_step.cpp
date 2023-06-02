/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif
#include "esp_err.h"
#include "iot_button.h"
#include <TMCStepper.h>
#include "driver_as5600_basic.h"


#define BUTTON_EN_PIN    GPIO_NUM_4 // Button to enable or disable the motor driver
#define EN_PIN           GPIO_NUM_23 // Enable
#define DIR_PIN          GPIO_NUM_18 // Direction
#define STEP_PIN         GPIO_NUM_19 // Step
// #define CS_PIN           42 // Chip select
// #define SW_MOSI          66 // Software Master Out Slave In (MOSI)
// #define SW_MISO          44 // Software Master In Slave Out (MISO)
// #define SW_SCK           64 // Software Slave Clock (SCK)
// #define SW_RX            63 // TMC2208/TMC2224 SoftwareSerial receive pin
// #define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);       // Hardware serial
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);     // Software serial

static const char *TAG_STEPPER = "Simple_Stepper";
static const char *TAG_ROTARY_SENSOR = "Rotary_Sensor";
static QueueHandle_t button_isr_queue = NULL;
button_handle_t g_btns[BUTTON_EN_PIN] = {0};
TaskHandle_t stepper_handler;
extern as5600_handle_t gs_handle;        /**< as5600 handle */


void button_single_click_cb(void *arg, void *data_to_send)
{
  xQueueSendFromISR(button_isr_queue, (void *) &data_to_send, NULL);
}

void setup_button() {
  button_config_t cfg = {
    .type = BUTTON_TYPE_GPIO,
    .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
    .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    .gpio_button_config = {
        .gpio_num = BUTTON_EN_PIN,
        .active_level = 0,
    },
  };
  g_btns[0] = iot_button_create(&cfg);

  // Create queue for button isr
  button_isr_queue = xQueueCreate(10, sizeof(gpio_num_t));

  // Register callback using button component
  iot_button_register_cb(g_btns[0], BUTTON_SINGLE_CLICK, button_single_click_cb, (void *) BUTTON_EN_PIN);
  
}

void startup() {

  as5600_info_t info;


  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;                            //disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT;                                  //set as output mode
  io_conf.pin_bit_mask = ((1ULL<<EN_PIN) | (1ULL<<STEP_PIN) | (1ULL<<DIR_PIN));        //bit mask of the pins that you want to set,e.g.GPIO18
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                     //disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                         //disable pull-up mode
  ESP_ERROR_CHECK(gpio_config(&io_conf));                          //configure GPIO with the given settings

  // Set to input and output mode, so can control enable pin voltage, and read its voltage.
  gpio_set_direction(EN_PIN, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_level(EN_PIN, 0);         // Enable driver in hardware

  // pinMode(EN_PIN, OUTPUT);       
  // pinMode(STEP_PIN, OUTPUT);
  // pinMode(DIR_PIN, OUTPUT);
  // digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

  // Configure Button ISR Pin
  setup_button();




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



  // gpio_config_t io_isr_conf;
  // io_isr_conf.intr_type = GPIO_INTR_NEGEDGE;                            //disable interrupt
  // io_isr_conf.mode = GPIO_MODE_INPUT;                                   //set as output mode
  // io_isr_conf.pin_bit_mask = (1ULL<<BUTTON_PIN);                        //bit mask of the pins that you want to set,e.g.GPIO18
  // io_isr_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                     //disable pull-down mode
  // io_isr_conf.pull_up_en = GPIO_PULLUP_ENABLE;                          //disable pull-up mode
  // ESP_ERROR_CHECK(gpio_config(&io_isr_conf));                           //configure GPIO with the given settings

  // gpio_install_isr_service(0);
  // gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, (void*) BUTTON_PIN);

                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  SERIAL_PORT.begin(115200);      // HW UART drivers
//driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(500);        // Set motor RMS current
  driver.microsteps(256);          // Set microsteps to 1/16th

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
}

// int accel;
// long maxSpeed;
// int speedChangeDelay;
// bool direction = false;

// void loop() {
//   // Run 5000 steps and switch direction in software
//   for (uint16_t i = 5000; i>0; i--) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(160);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(160);
//   }
//   direction = !direction;
//   driver.shaft(direction);
//   ESP_LOGI(TAG_STEPPER, "Changing direction!!!");
// }

void stepper_motor_task(void *parameter) {

  int accel = 100000;                                         // Speed increase/decrease amount
  long maxSpeed = 250000;                                     // Maximum speed to be reached
  int speedChangeDelay = 100;                                 // Delay between speed changes
  bool direction = false;                                     // Direction flag


  
  while (true) {

    for (long i = 0; i <= maxSpeed; i = i + (accel/2)){             // Speed up to maxSpeed
      driver.VACTUAL(i);                                     // Set motor speed
      vTaskDelay(speedChangeDelay / portTICK_PERIOD_MS);


    }

    // Maintain speed for some time
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    for (long i = maxSpeed; i >=0; i = i - accel){              // Decrease speed to zero
      driver.VACTUAL(i);
      vTaskDelay(speedChangeDelay / portTICK_PERIOD_MS);
    }  
    
    direction = !direction; // REVERSE DIRECTION
    driver.shaft(direction); // SET DIRECTION
    ESP_LOGI(TAG_STEPPER, "Changing direction using VACTUAL!");

  }

}

void driver_enable_check_task(void *params)
{
  int data_received;
  int count = 0;
  while (true)
  {
    if (xQueueReceive(button_isr_queue, &data_received, portMAX_DELAY))
    {
      ESP_LOGI(TAG_STEPPER, "GPIO %d was pressed %d times. The state is %d\n", data_received, ++count, gpio_get_level(BUTTON_EN_PIN));

      // Switch the driver on or off (0=ON, 1=OFF)
      int new_level = (gpio_get_level(EN_PIN) + 1) % 2;
      ESP_LOGD(TAG_STEPPER, "new_level to set is: %d\n", new_level);
      gpio_set_level(EN_PIN, new_level);
      switch (new_level) {
        case 0:
          vTaskResume(stepper_handler);
          break;
        case 1:
          vTaskSuspend(stepper_handler);
          break;
      }
      ESP_LOGI(TAG_STEPPER, "EN_PIN level is: %d\n", gpio_get_level(EN_PIN));
    }
  }
}

void rotary_sensor_task(void *params)
{
  uint8_t AS5600_status;
  uint8_t AS5600_agc;
  uint16_t AS5600_magnitude;
  float deg;                                                  // degree of the stepper motor

  uint16_t start_pos;                                         // start position of AS5600
  uint16_t stop_pos;                                          // stop position of AS5600
  uint16_t max_angle;                                         // max angle allowed on AS5600


  static uint16_t set_start_pos = 2;
  //static uint16_t set_stop_pos = 2000;
  static uint16_t set_max_angle = 280;

  ESP_ERROR_CHECK(as5600_set_start_position(&gs_handle, set_start_pos));
  //ESP_ERROR_CHECK(as5600_set_stop_position(&gs_handle, set_stop_pos));
  as5600_set_max_angle(&gs_handle, set_max_angle);

  while (true) {
    // Get AS5600 chip status
    as5600_get_status(&gs_handle, &AS5600_status);

    if (AS5600_status & AS5600_STATUS_MD) {
      ESP_LOGI(TAG_ROTARY_SENSOR, "Magnet detected.\n");

      if (AS5600_status & AS5600_STATUS_MH) {
        ESP_LOGW(TAG_ROTARY_SENSOR, "Magnet is too strong.\n");
      }
      else if (AS5600_status & AS5600_STATUS_ML) {
        ESP_LOGW(TAG_ROTARY_SENSOR, "Magnet is too weak.\n");
      }
      else {
        ESP_LOGD(TAG_ROTARY_SENSOR, "Magnet is just right :)\n");
      }
    }
    else {
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

}


extern "C" void app_main() {


  initArduino();
  startup();

  xTaskCreatePinnedToCore(driver_enable_check_task, "Driver Enable Check Task", 4096, NULL, 4, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(stepper_motor_task, "Stepper Motor Task", 4096, NULL, 3, &stepper_handler, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(rotary_sensor_task, "Rotary Sensor Task", 4096, NULL, 3, NULL, tskNO_AFFINITY);

  while (1) {
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }

  iot_button_delete(g_btns[0]);
  (void)as5600_basic_deinit();

}