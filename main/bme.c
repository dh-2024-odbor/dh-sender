#include "bme.h"
#include "config.h"
#include "esp_log.h"

esp_err_t bme280_i2c_init() {
    ESP_LOGI(TAG, "initializing i2c in mode master");

    // Since BME280 uses I2C for communication
    // we specify the init struct 
    // and provide necessary timing information
    // and gpio

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLDOWN_ENABLE,
        .scl_pullup_en = GPIO_PULLDOWN_ENABLE,
        .master.clk_speed = 1000000
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &i2c_config);

    if (err != ESP_OK) {
      return err;
    }

    // Install the driver in mode master to I2C port 0
    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t bme280_sensor_init() {
  s32 com_rslt;

  // Initialize the sensor internal structures
  com_rslt = bme280_init(&s_bme280);

  // Calibrate the sensor 
  com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

  // Put the sensor into Forced mode
  // to lower the power consumption
  
	com_rslt += bme280_set_power_mode(BME280_FORCED_MODE);

  if (com_rslt == SUCCESS) {
    return ESP_OK;
  }

  return ESP_FAIL;
}