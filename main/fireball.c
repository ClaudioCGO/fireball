#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "board.h"
#include "motors.h"
#include "pwm.h"
#include "tcrt5000.h"

#define FINISH_TIME 25

#define MAX_SPEED 1200
#define MED_SPEED 1000
#define TURN_SPEED 1200

#define THRESHOLD_LE 2200 // W = 1200; B = 3150; T = 2200
#define THRESHOLD_LI 1220 // W = 560; B = 1880; T = 1220
#define THRESHOLD_RI 800  // W = 360; B = 1250; T = 800
#define THRESHOLD_RE 2100 // W = 1200; B = 3100; T = 2100

typedef enum {
    TRACK_HARD_LEFT     = -2,
    TRACK_SMALL_LEFT    = -1,
    TRACK_CENTER        = 0,
    TRACK_SMALL_RIGHT   = 1,
    TRACK_HARD_RIGHT    = 2,
} track_state_t;

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


static track_state_t calculate_track_error (sensor_data_t data, track_state_t current_last_error) {
    if (data.left_ext && data.right_ext) return current_last_error;

    if (data.left_ext) return TRACK_HARD_LEFT;
    if (data.right_ext) return TRACK_HARD_RIGHT;
    if (data.left_int && !data.right_int) return TRACK_SMALL_LEFT;
    if (!data.left_int && data.right_int) return TRACK_SMALL_RIGHT;
    if (data.left_int && data.right_int) return TRACK_CENTER;

    return current_last_error;
}


void control_task (void *pvParameters) {
    sensor_data_t data;
    track_state_t current_state = TRACK_CENTER;
    int finish_counter = 0;
   
    while (!finished) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY) != pdPASS) {
            continue;
        }        

        if (data.left_ext && data.left_int && data.right_int && data.right_ext) {
            finish_counter++;

            ESP_LOGD(TAG, "All black detected | Count: %d", finish_counter);
            motors_forward();
            set_PWM(MAX_SPEED, MAX_SPEED);
        
            if (finish_counter >= FINISH_TIME) {
                ESP_LOGI(TAG, "Track Ended!");
                finished = true;
                break;
            }
            continue;
        }

        finish_counter = 0;

        current_state = calculate_track_error(data, current_state);

        switch (current_state) {
            case TRACK_HARD_LEFT:
                ESP_LOGD(TAG, "HARD TURN LEFT");
                motors_yaw_left();
                set_PWM(TURN_SPEED, TURN_SPEED);
                break;

            case TRACK_HARD_RIGHT:
                ESP_LOGD(TAG, "HARD TURN RIGHT");
                motors_yaw_right();
                set_PWM(TURN_SPEED, TURN_SPEED);
                break;

            case TRACK_SMALL_LEFT:
                ESP_LOGD(TAG, "SMALL TURN LEFT");
                motors_forward();
                set_PWM(MED_SPEED, MAX_SPEED);
                break;

            case TRACK_SMALL_RIGHT:
                ESP_LOGD(TAG, "SMALL TURN RIGHT");
                motors_forward();
                set_PWM(MAX_SPEED, MED_SPEED);
                break;

            case TRACK_CENTER:
                ESP_LOGD(TAG, "STRAIGHT");
                motors_forward();
                set_PWM(MAX_SPEED, MAX_SPEED);
                break;

            default:
                ESP_LOGW(TAG, "UNHANDLED STATE, BRAKING");
                motors_brake();
                set_PWM(0, 0);
                break;
        }
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