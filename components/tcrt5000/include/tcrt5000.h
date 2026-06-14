#pragma once
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

void init_adc_channels(void);
esp_err_t ir_read(int *le, int *li, int *ri, int *re);
