#include "lora.h"
#include "bme.h"
#include <stdio.h>
#include "driver/rtc_io.h"
#include <sys/time.h>
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"

// Wakeup constants

#define uS_TO_S_FACTOR 1000000
#define WAKEUP_PERIOD_S 5

// The ID of the node in the network
// arbitary value as long as it is unique
// within the network 
// Identifies the device to the backend

#define NODE_ID (uint32_t) 9

// The boot count which is stored
// in RTC memory
// which is the part of flash which runs
// even when the device is in deep sleep
// we use this value as a message
// id to guarantee uniqueness

static RTC_DATA_ATTR uint32_t boot_count;

// Buffer for the data that is transmitted
// via LoRa

static uint8_t buf[20];

void deep_sleep_task() {
   ESP_LOGI("main", "device just wokeup");
   // We read the wakeup reason to determine if the RTC timer
   // woke us up
   // (technically we do not need that because we only have
   // one singular wakeup cause)

  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();

   // Check if the wakeup reason is that timer

   if (reason == ESP_SLEEP_WAKEUP_TIMER) {
      // We woke up from a RTC timer
      // lets transmit the data and go back to sleep

      s32 v_uncomp_pressure;
      s32 v_uncomp_temperature;
      s32 v_uncomp_humidity;

      // Read values from the bme280 sensor
      // The read values are uncompensated
      // because that is how the sensor sends them
      // we need to "convert" them into
      // values that have meaning for us

      s32 com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
         &v_uncomp_pressure, &v_uncomp_temperature, &v_uncomp_humidity
      );

      // Check if the values were successfully read
      // convert them into our format and transmit 
      // the JSON serialized packet containing them

      if (com_rslt == SUCCESS) {
         float temperature = (float) bme280_compensate_temperature_double(v_uncomp_temperature);
         float pressure = (float) bme280_compensate_pressure_double(v_uncomp_pressure) / 100;
         float humidity = (float) bme280_compensate_humidity_double(v_uncomp_humidity);


         // This is the message format for this device
         // The two enforced fields
         // and NODE_ID which identifies the device
         // within the network
         // and a message_id which is derived
         //  from the boot_counter
         // which is the number of times
         // the device entered deep sleep

         *buf = NODE_ID;
         *(buf + 4) = boot_count;

         // This is the arbitrary data part,
         // format for which is defined on the server.
         // Each NODE_ID has a specific data format
         // which the server knows and uses it to
         // extract data from the received "buf"

         *((float *) (buf + 8)) = temperature;
         *((float *) (buf + 12)) = pressure;
         *((float *) (buf + 16)) = humidity;

         ESP_LOGI("main", "Boot count: %d", boot_count++);

         lora_send_packet((uint8_t*) buf, 20);
      }
   }

   // We isolate this pin due to an
   // external pull-up resistor
   // to minimize current consumption
   // when deep-sleeping
   // Needed because the SX1278
   // uses one of these pins
   // which remain connected when if device sleeps
   rtc_gpio_isolate(GPIO_NUM_12);

   // Put the bme280 sensor into
   // sleep mode so it consumes less current

   // FORCED MODE automatically puts the
   // device to sleep after a read is requested

   // bme280_set_power_mode(BME280_SLEEP_MODE);

   // Put the SX1278 transciever into
   // sleep mode again to lower the overall
   // power consumption
   lora_sleep();

   // Turn off the RTC IO and the ULP co-processor
   // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_OFF);

   // Since we do not need any memory while we are in deep sleep
   // we turn off both slow and fast RTC memory
   //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
   //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);

   // vTaskDelay(1000 / portTICK_PERIOD_MS);


   // Turn off the XTAL oscillator

   int64_t bigi = esp_timer_get_time();
   

   ESP_LOGW("main", "Awake time:  %" PRId32 "%" PRId32 " microseconds\n", (int)(bigi >> 32), (int)bigi);
   ESP_LOGI("main", "Entering deep-sleep");
   esp_deep_sleep_start();
}

void app_main()
{
   ESP_LOGI("main", "Device exited deep-sleep");
   // Enable the RTC timer
   // which runs when the device is in deep sleep mode
   // and wakes up the device when the specified
   // amount of time elapses

   ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * WAKEUP_PERIOD_S));

   // Initialize i2c driver for the bme280 sensor
   ESP_ERROR_CHECK(bme280_i2c_init());

   // Initialize the bme280 sensor
   ESP_ERROR_CHECK(bme280_sensor_init());

   // Initialite lora radio module
   ESP_ERROR_CHECK(lora_initialize_radio());

   // bme280 sensor testing

   /* while (true) {
      s32 v_uncomp_temperature;
      s32 v_uncomp_humidity;
      s32 v_uncomp_pressure;


      s32 com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
         &v_uncomp_pressure, &v_uncomp_temperature, &v_uncomp_humidity
      );

      if (com_rslt == SUCCESS) {
         double temperature = bme280_compensate_temperature_double(v_uncomp_temperature);
         double pressure = bme280_compensate_pressure_double(v_uncomp_pressure);
         double humidity = bme280_compensate_humidity_double(v_uncomp_humidity);

         char buf[80];
         
         ssize_t bufsiz = sprintf(buf, "{\"t\": %f, \"h\": %f, \"p\": %f }", temperature, humidity, pressure);

         ESP_LOGI("main", "Received %s", buf);
      }

      vTaskDelay(5000 / portTICK_PERIOD_MS);
   } */

   

   // Create the task which reads data from sensors
   // transmits it then returns all devices to low-consumption
   // mode.

   xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);
}