#include "lora.h"
#include "bme.h"
#include <stdio.h>
#include "driver/rtc_io.h"
#include <sys/time.h>
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

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

// static RTC_DATA_ATTR uint32_t boot_count;

static nvs_handle_t s_nvs_handle;

// Buffer for the data that is transmitted
// via LoRa

static uint8_t buf[20];

void deep_sleep_task() {
   esp_err_t ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
   }
   ESP_ERROR_CHECK( ret );

   ret = nvs_open("storage", NVS_READWRITE, &s_nvs_handle);

   ESP_ERROR_CHECK( ret );

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
      // them as a simple buffer

      if (com_rslt == SUCCESS) {
         float temperature = (float) bme280_compensate_temperature_double(v_uncomp_temperature);
         float pressure = (float) bme280_compensate_pressure_double(v_uncomp_pressure) / 100;
         float humidity = (float) bme280_compensate_humidity_double(v_uncomp_humidity);

         uint32_t boot_count;

         // Read the boot_cout from the NVS (non-volatile storage)
         // AKA permanent storage which keeps data
         // after power cycle
         uint32_t ret = nvs_get_u32(s_nvs_handle, "boot_count", &boot_count);

         // If the value does not exist yet
         // in the NVS default it to 1
         if (ret == ESP_ERR_NVS_NOT_FOUND) {
            boot_count = 1;
         }      

         ESP_LOGI("main", "Boot count: %d", boot_count);

         // This is the message format for this device
         // The two enforced fields
         // and NODE_ID which identifies the device
         // within the network
         // and a message_id which is derived
         //  from the boot_counter
         // which is the number of times
         // the device entered deep sleep

         *((uint32_t *) buf) = NODE_ID;
         *((uint32_t *) (buf + 4)) = boot_count;

         // This is the arbitrary data part,
         // format for which is defined on the server.
         // Each NODE_ID has a specific data format
         // which the server knows and uses it to
         // extract data from the received "buf"

         *((float *) (buf + 8)) = temperature;
         *((float *) (buf + 12)) = pressure;
         *((float *) (buf + 16)) = humidity;

         lora_send_packet((uint8_t*) buf, 20);

         // Save the boot_count to the NVS
         // in the rare case the device resets
         // or restarts, so ensure the
         // message_ids remain unique

         nvs_set_u32(s_nvs_handle, "boot_count", boot_count + 1);
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

   // NOTE: Forced mode for BME280 automatically puts the
   // device to sleep after a read is requested
   // which means that the below line is unnecessary

   // bme280_set_power_mode(BME280_SLEEP_MODE);

   // Put the SX1278 transciever into
   // sleep mode again to lower the overall
   // power consumption
   lora_sleep();

   // Read the time from boot
   // to determine how much time the
   // device is up before going back to sleep

   int64_t elapsed_from_boot = esp_timer_get_time();

   ESP_LOGW("main", "Awake time:  %" PRId32 "%" PRId32 " microseconds\n", (int)(elapsed_from_boot >> 32), (int)elapsed_from_boot);
   ESP_LOGI("main", "Entering deep-sleep");

   // Enter the deep sleep
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