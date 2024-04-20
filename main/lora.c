#include "lora.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

esp_err_t lora_initialize_radio() {
  // Initializes the driver code
  // which facilitates communication
  // with the SX1278 transciever. 
  // The transicever supports LoRa modulation technique
  // and communicates with the MCU via SPI

  lora_init();

  // Sets the operation radio frequency of the transciever
  // We are using the unlicensed frequency band
  // 433e6 which is standard in the EU

  lora_set_frequency(LORA_FREQ);

  // Sets the additional radio
  // parameters to optimize for power consumption
  // while balancing time in the air and 
  // and maximum transmission distance

  lora_set_bandwidth(125E6);
  lora_set_spreading_factor(12);
  lora_set_coding_rate(5);
  lora_set_preamble_length(8);
  lora_set_tx_power(13);

  // Enables CRC checking which
  // ensures received package integrity
  // The radio automatically rejects
  // the packet if it sees
  // that the CRC value is incorrect

  lora_enable_crc();

  return ESP_OK;  
}

