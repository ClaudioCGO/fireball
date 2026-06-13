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

#define THRESHOLD_LE 900
#define THRESHOLD_LI 720
#define THRESHOLD_RI 400
#define THRESHOLD_RE 1480

#define FINISH_TIME 100

#define MAX_SPEED 2000
#define MED_SPEED 1800
#define TURN_SPEED 2000

typedef struct {
    bool b_le;
    bool b_li;
    bool b_ri;
    bool b_re;
} sensor_data_t;

QueueHandle_t sensor_queue;

static volatile bool finished = false;


void sensor_task(void *pvParameters) {
    int le, li, ri, re;
    sensor_data_t current_reading;

    while(!finished) {
        ir_read(&le, &li, &ri, &re);

        current_reading.b_le = le > THRESHOLD_LE;
        current_reading.b_li = li > THRESHOLD_LI;
        current_reading.b_ri = ri > THRESHOLD_RI;
        current_reading.b_re = re > THRESHOLD_RE;

        xQueueSend(sensor_queue, &current_reading, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

void control_task(void *pvParameters) {
    sensor_data_t data;
    int last_error = 0;
    int finish_counter = 0;

    while(!finished) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (data.b_le && data.b_li && data.b_ri && data.b_re) {
            finish_counter++;

            printf("DO NOTHING and go forward... %d    ", finish_counter);
            motors_forward();
            set_PWM(MAX_SPEED, MAX_SPEED);

            if (finish_counter >= FINISH_TIME) {
                motors_brake();
                set_PWM(0, 0);
                finished = true;
                printf("Ended!");
            }
            continue;
        }


        finish_counter = 0;


        if (data.b_le) {
            printf("HARD TURN LEFT   ");
            motors_yaw_left();
            set_PWM(TURN_SPEED, TURN_SPEED);
            last_error = -2;
            continue;
        }

        if (data.b_re) {
            printf("HARD TURN RIGHT   ");
            motors_yaw_right();
            set_PWM(TURN_SPEED, TURN_SPEED);
            last_error = 2;
            continue;
        }

        if (data.b_li && !data.b_ri) {
            printf("SMALL TURN LEFT   ");
            motors_forward();
            set_PWM(MED_SPEED, MAX_SPEED);
            last_error = -1;
            continue;
        }

        if (!data.b_li && data.b_ri) {
            printf("SMALL TURN RIGHT   ");
            motors_forward();
            set_PWM(MAX_SPEED, MED_SPEED);
            last_error = 1;
            continue;
        }

        if (data.b_li && data.b_ri) {
            printf("STRAIGHT   ");
            motors_forward();
            set_PWM(MAX_SPEED, MAX_SPEED);
            continue;
        }


        // Searching Logic
        if (last_error < 0) {
            printf("SEARCHING LEFT   ");
            motors_yaw_left();
            set_PWM(TURN_SPEED, TURN_SPEED);
        }
        else if (last_error > 0) {
            printf("SEARCHING RIGHT   ");
            motors_yaw_right();
            set_PWM(TURN_SPEED, TURN_SPEED);     
        }
        else {
            printf("NOTHING TO SEARCH   ");
            motors_brake();
            set_PWM(0, 0);
        }
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

    printf("WAITING 3 SECONDS...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    sensor_queue = xQueueCreate(5, sizeof(sensor_data_t));

    if (sensor_queue != NULL) {
        xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 4, NULL);
    } else {
        printf("Failed to create Queue!\n");
    }
}
