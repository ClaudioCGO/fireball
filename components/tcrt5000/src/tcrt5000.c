#include "tcrt5000.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"

static const char* TAG = "Tcrt5000";

static adc_oneshot_unit_handle_t adc1_handle;

static adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};

static adc_oneshot_chan_cfg_t config = {
  .bitwidth = ADC_BITWIDTH_DEFAULT,
  .atten = ADC_ATTEN_DB_12,
};

void init_adc_channels(void)
{ 
  /* Check if the unit was sucessfull created */
  esp_err_t err = adc_oneshot_new_unit(&init_config, &adc1_handle);
  
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
    return;
  }

  /* Check if the channel 0 was sucessfull created */
  err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config);

  if (err != ESP_OK)
  { 
    ESP_LOGE(TAG, "adc_oneshot_config_channel of channel 0 failed: %s", esp_err_to_name(err));
    return;
  }

  /* Check if the channel 1 was sucessfull created */
  err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config);

  if (err != ESP_OK)
  { 
    ESP_LOGE(TAG, "chan0: %s", esp_err_to_name(err));
    return;
  }

  /* Check if the channel 3 was sucessfull created */
  err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config);

  if (err != ESP_OK)
  { 
    ESP_LOGE(TAG, "chan0: %s", esp_err_to_name(err));
    return;
  }

  /* Check if the channel 4 was sucessfull created */
  err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config);

  if (err != ESP_OK)
  { 
    ESP_LOGE(TAG, "chan0: %s", esp_err_to_name(err));
    return;
  }
}

void ir_read(int *le, int *li, int *ri, int *re)
{
  adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, le);
  adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, li);
  adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, re);
  adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, ri);
  
  ESP_LOGI(TAG,"LE: %d | LI: %d | RI: %d | RE: %d", *le, *li, *ri, *re);
}
