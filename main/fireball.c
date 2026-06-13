#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board.h"
#include "motors.h"
#include "pwm.h"
#include "tcrt5000.h"

#define FINISH_TIME 100
#define MAX_SPEED 1200
#define MED_SPEED 1000
#define TURN_SPEED 1200

#define THRESHOLD_LE 2200 // W = 1200; B = 3150; T = 2200
#define THRESHOLD_LI 1220 // W = 560; B = 1880; T = 1220
#define THRESHOLD_RI 800  // W = 360; B = 1250; T = 800
#define THRESHOLD_RE 2100 // W = 1200; B = 3100; T = 2100

static const char *TAG = "fireball";

static int le, li, ri, re;
static int last_error = 0;
static int finish_counter = 0;
static volatile bool finished = false;

void drive_task(void *pvParameters)
{
  while (!finished)
  {
    ir_read(&le, &li, &ri, &re);

    bool b_le = le > THRESHOLD_LE;
    bool b_li = li > THRESHOLD_LI;
    bool b_ri = ri > THRESHOLD_RI;
    bool b_re = re > THRESHOLD_RE;


    if (b_le && b_li && b_ri && b_re) {
      finish_counter++;

      ESP_LOGI(TAG, "DO NOTHING AND ACCELERATE | Count: %d", finish_counter);
      motors_forward();
      set_PWM(MAX_SPEED, MAX_SPEED);
      
      if (finish_counter >= FINISH_TIME) {
        motors_brake();
        set_PWM(0, 0);
        finished = true;
        ESP_LOGI(TAG, "Ended!");
      }
    }
    else
    {
      finish_counter = 0;

      if (b_le)
      {
        motors_yaw_left();
        set_PWM(TURN_SPEED, TURN_SPEED);
        last_error = -2;
        ESP_LOGI(TAG, "HARD TURN LEFT");
      }
      else
      if (b_re)
      {
        motors_yaw_right();
        set_PWM(TURN_SPEED, TURN_SPEED);
        last_error = 2;
        ESP_LOGI(TAG, "HARD TURN RIGHT");
      }
      else
      if (b_li && !b_ri)
      {
        motors_forward();
        set_PWM(MED_SPEED, MAX_SPEED);
        last_error = -1;
        ESP_LOGI(TAG, "SMALL TURN LEFT");
      }
      else
      if (!b_li && b_ri)
      {
        motors_forward();
        set_PWM(MAX_SPEED, MED_SPEED);
        last_error = 1;
        ESP_LOGI(TAG, "SMALL TURN RIGHT");
      }
      else
      if (b_li && b_ri)
      {
        motors_forward();
        set_PWM(MAX_SPEED, MAX_SPEED);
        ESP_LOGI(TAG, "STRAIGHT");
      }
      else
      if (last_error < 0)
      {
        motors_yaw_left();
        set_PWM(TURN_SPEED, TURN_SPEED);
        ESP_LOGI(TAG, "SEARCHING LEFT");
      }
      else
      if (last_error > 0)
      {
        motors_yaw_right();
        set_PWM(TURN_SPEED, TURN_SPEED);
        ESP_LOGI(TAG, "SEARCHING RIGHT");
      }
      else
      {
        motors_brake();
        set_PWM(0, 0);
        ESP_LOGI(TAG, "NOTHING TO SEARCH");
      }
    }
    ESP_LOGI(TAG,"LE: %d | LI: %d | RI: %d | RE: %d", le, li, ri, re);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete(NULL);
}

void app_main(void)
{
    motors_control_init();

    pwm_config_t pwm_cfg = {
        .left_pin = HB_PWMA,
        .right_pin = HB_PWMB
    };
    configure_pwm(pwm_cfg);

    init_adc_channels();

    ESP_LOGI(TAG, "WAITING 3 SECONDS...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    xTaskCreate(drive_task, "drive_task", 4096, NULL, 5, NULL);
}
