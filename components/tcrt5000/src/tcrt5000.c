#include "tcrt5000.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"

static const char* TAG = "Tcrt5000";

static adc_oneshot_unit_handle_t adc1_handle = NULL;

static adc_oneshot_unit_init_cfg_t init_config = {
  .unit_id = ADC_UNIT_1,
  .ulp_mode = ADC_ULP_MODE_DISABLE,
};

static adc_oneshot_chan_cfg_t config = {
  .bitwidth = ADC_BITWIDTH_DEFAULT,
  .atten = ADC_ATTEN_DB_12,
};

static const adc_channel_t active_channels[] = {
  ADC_CHANNEL_0,
  ADC_CHANNEL_1,
  ADC_CHANNEL_3,
  ADC_CHANNEL_4
};
static const int num_channels = sizeof(active_channels) / sizeof(active_channels[0]);


void init_adc_channels(void) {
  if (adc1_handle != NULL) {
    ESP_LOGW(TAG, "ADC unit is already initialized!");
    return;
  }

  esp_err_t err = adc_oneshot_new_unit(&init_config, &adc1_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
    return;
  }

  for (int i = 0; i < num_channels; i++) {
    err = adc_oneshot_config_channel(adc1_handle, active_channels[i], &config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure ADC channel %d: %s", active_channels[i], esp_err_to_name(err));
      return;
    }
  }

  ESP_LOGI(TAG, "TCRT5000 ADC channels initiated successfully.");
}


esp_err_t ir_read(int *le, int *li, int *ri, int *re) {
  if (adc1_handle == NULL) {
    ESP_LOGE(TAG, "Cannot read IR sensors: ADC not initialized.");
    return ESP_ERR_INVALID_STATE;
  }
  
  esp_err_t err;
  
  err = adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, le);
  if (err != ESP_OK) return err;

  err = adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, li);
  if (err != ESP_OK) return err;
  
  err = adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, re);
  if (err != ESP_OK) return err;
  
  err = adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, ri);
  if (err != ESP_OK) return err;

  return ESP_OK;
}
