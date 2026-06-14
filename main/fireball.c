#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

typedef struct {
    bool b_le;
    bool b_li;
    bool b_ri;
    bool b_re;
} sensor_data_t;

QueueHandle_t sensor_queue;

static const char *TAG = "fireball";

static volatile bool finished = false;

void sensor_task (void *pvParameters) {
    int le, li, ri, re;
    sensor_data_t current_reading;

    while (!finished) {
        ir_read(&le, &li, &ri, &re);

        current_reading.b_le = le > THRESHOLD_LE;
        current_reading.b_li = li > THRESHOLD_LI;
        current_reading.b_ri = ri > THRESHOLD_RI;
        current_reading.b_re = re > THRESHOLD_RE;

        xQueueOverwrite(sensor_queue, &current_reading);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void control_task (void *pvParameters) {
    sensor_data_t data;
    int last_error = 0;
    int finish_counter = 0;
   
    while (!finished) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY) != pdPASS) {
            continue;
        }        

        if (data.b_le && data.b_li && data.b_ri && data.b_re) {
            finish_counter++;

            ESP_LOGI(TAG, "DO NOTHING AND ACCELERATE | Count: %d", finish_counter);
            motors_forward();
            set_PWM(MAX_SPEED, MAX_SPEED);
        
            if (finish_counter >= FINISH_TIME) {
                ESP_LOGI(TAG, "Ended!");
                motors_brake();
                set_PWM(0, 0);
                finished = true;
            }
            continue;
        }

        finish_counter = 0;

        if (data.b_le) {
            ESP_LOGI(TAG, "HARD TURN LEFT");
            motors_yaw_left();
            set_PWM(TURN_SPEED, TURN_SPEED);
            last_error = -2;
            continue;
        }

        if (data.b_re) {
            ESP_LOGI(TAG, "HARD TURN RIGHT");
            motors_yaw_right();
            set_PWM(TURN_SPEED, TURN_SPEED);
            last_error = 2;
            continue;
        }

        if (data.b_li && !data.b_ri) {
            ESP_LOGI(TAG, "SMALL TURN LEFT");
            motors_forward();
            set_PWM(MED_SPEED, MAX_SPEED);
            last_error = -1;
            continue;
        }

        if (!data.b_li && data.b_ri) {
            ESP_LOGI(TAG, "SMALL TURN RIGHT");
            motors_forward();
            set_PWM(MAX_SPEED, MED_SPEED);
            last_error = 1;
            continue;
        }

        if (data.b_li && data.b_ri) {
            ESP_LOGI(TAG, "STRAIGHT");
            motors_forward();
            set_PWM(MAX_SPEED, MAX_SPEED);
            continue;
        }

        if (last_error < 0) {
            ESP_LOGI(TAG, "SEARCHING LEFT");
            motors_yaw_left();
            set_PWM(TURN_SPEED, TURN_SPEED);
            continue;
        }

        else if (last_error > 0) {
            ESP_LOGI(TAG, "SEARCHING RIGHT");
            motors_yaw_right();
            set_PWM(TURN_SPEED, TURN_SPEED);
        }
        
        else {
            ESP_LOGI(TAG, "NOTHING TO SEARCH");
            motors_brake();
            set_PWM(0, 0);
        }
    }
  vTaskDelete(NULL);
}

void app_main (void) {
    motors_control_init();

    pwm_config_t pwm_cfg = {
        .left_pin = HB_PWMA,
        .right_pin = HB_PWMB
    };
    configure_pwm(pwm_cfg);

    init_adc_channels();

    ESP_LOGI(TAG, "WAITING 3 SECONDS...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    sensor_queue = xQueueCreate(1, sizeof(sensor_data_t));

    if (sensor_queue == NULL) {
        ESP_LOGI(TAG, "FAILED TO CREATE QUEUE!");
    }

    else {
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 4, NULL);
        xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
    }
}