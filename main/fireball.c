#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "board.h"
#include "motors.h"
#include "pwm.h"
#include "tcrt5000.h"
#include "pid.h"

#define FINISH_TIME 25

#define BASE_SPEED 1200
#define MAX_SPEED 3000

#define THRESHOLD_LE 2200 // W = 1200; B = 3150; T = 2200
#define THRESHOLD_LI 1220 // W = 560; B = 1880; T = 1220
#define THRESHOLD_RI 800  // W = 360; B = 1250; T = 800
#define THRESHOLD_RE 2100 // W = 1200; B = 3100; T = 2100

#define KP 15.0f
#define KI 0.0f
#define KD 15.0f

typedef struct {
    bool left_ext;
    bool left_int;
    bool right_int;
    bool right_ext;
} sensor_data_t;

QueueHandle_t sensor_queue;
static const char *TAG = "fireball";
static volatile bool finished = false;


void sensor_task (void *pvParameters) {
    int le, li, ri, re;
    sensor_data_t current_reading;

    while (!finished) {
        if (ir_read(&le, &li, &ri, &re) == ESP_OK) {
            current_reading.left_ext  = le > THRESHOLD_LE;
            current_reading.left_int  = li > THRESHOLD_LI;
            current_reading.right_int = ri > THRESHOLD_RI;
            current_reading.right_ext = re > THRESHOLD_RE;

            xQueueOverwrite(sensor_queue, &current_reading);
        } else {
            ESP_LOGE(TAG, "IR read failed. Skipping this cycle.");
        }

        ESP_LOGD(TAG,"LE: %d | LI: %d | RI: %d | RE: %d", le, li, ri, re);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}


static float calculate_track_error (sensor_data_t data, float current_last_error) {
    if (data.left_ext && data.right_ext) return current_last_error;

    if (data.left_ext) return -20.0f;
    if (data.right_ext) return 20.0f;
    if (data.left_int && !data.right_int) return -10.0f;
    if (!data.left_int && data.right_int) return 10.0f;
    if (data.left_int && data.right_int) return 0.0f;

    return current_last_error;
}


void control_task (void *pvParameters) {
    sensor_data_t data;
    float current_error = 0.0f;
    int finish_counter = 0;
   
    pid_controller_t car_pid;
    pid_init(&car_pid, KP, KI, KD);

    while (!finished) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY) != pdPASS) {
            continue;
        }        

        if (data.left_ext && data.left_int && data.right_int && data.right_ext) {
            finish_counter++;

            ESP_LOGD(TAG, "All black detected | Count: %d", finish_counter);
            motors_forward();
            set_PWM(BASE_SPEED, BASE_SPEED);
        
            if (finish_counter >= FINISH_TIME) {
                ESP_LOGI(TAG, "Track Ended!");
                finished = true;
                break;
            }
            continue;
        }

        finish_counter = 0;

        current_error = calculate_track_error(data, current_error);

        float adjustment = pid_compute(&car_pid, current_error);

        int left_pwm = BASE_SPEED + adjustment;
        int right_pwm = BASE_SPEED - adjustment;

        if (left_pwm > MAX_SPEED) left_pwm = MAX_SPEED;
        if (left_pwm < 0) left_pwm = 0; 
        
        if (right_pwm > MAX_SPEED) right_pwm = MAX_SPEED;
        if (right_pwm < 0) right_pwm = 0;

        ESP_LOGD(TAG, "Err: %.1f | Adj: %.1f | L: %d | R: %d", current_error, adjustment, left_pwm, right_pwm);
        motors_forward();
        set_PWM(left_pwm, right_pwm);
    }
    
    motors_brake();
    set_PWM(0, 0);
    ESP_LOGI(TAG, "Control task exiting.");
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
        ESP_LOGE(TAG, "FAILED TO CREATE QUEUE! Out of heap memory.");
        return;
    }

    else {
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
        xTaskCreate(control_task, "control_task", 4096, NULL, 4, NULL);
    }
}