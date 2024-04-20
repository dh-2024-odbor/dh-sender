#ifndef _LORA_H_
#define _LORA_H_

#include "sx1278.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LORA_FREQ 433e6

esp_err_t lora_initialize_radio();

#endif