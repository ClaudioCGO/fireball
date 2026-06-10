#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "board.h"
#include "motors.h"
#include "pwm.h"
#include "tcrt5000.h"

#define THRESHOLD 2000
#define FINISH_TIME 35

#define MAX_SPEED 3500 
#define MED_SPEED 2000 
#define TURN_SPEED 2500 


void app_main(void)
{
    motors_control_init();

    pwm_config_t pwm_cfg = {
        .left_pin = HB_PWMA,
        .right_pin = HB_PWMB
    };
    configure_pwm(pwm_cfg);

    init_adc_channels();

    int le, li, ri, re;
    int last_error = 0; 
    int finish_counter = 0;

    bool finished = false;

    while (!finished)
    {
        ir_read(&le, &li, &ri, &re);

        bool b_le = le < THRESHOLD;
        bool b_li = li < THRESHOLD;
        bool b_ri = ri < THRESHOLD;
        bool b_re = re < THRESHOLD;

        if (b_le && b_li && b_ri && b_re) {
            finish_counter++;
            if (finish_counter >= FINISH_TIME) {
                motors_brake();
                set_PWM(0, 0);
                finished = true;
                printf("Ended!");
            }
        }

        else {
            finish_counter = 0;

            if (b_le) {
                motors_yaw_left();
                set_PWM(TURN_SPEED, TURN_SPEED);
                last_error = -2;
            }

            else if (b_re) {
                motors_yaw_right();
                set_PWM(TURN_SPEED, TURN_SPEED);
                last_error = 2;
            }

            else if (b_li && !b_ri) {
                motors_forward();
                set_PWM(MED_SPEED, MAX_SPEED);
                last_error = -1;
            }

            else if (!b_li && b_ri) {
                motors_forward();
                set_PWM(MAX_SPEED, TURN_SPEED);
                last_error = 1;
            }

            else if (b_li && b_re) {
                motors_forward();
                set_PWM(MAX_SPEED, MAX_SPEED);
                last_error = 0;
            }

            else {
                if (last_error < 0) {
                    motors_yaw_left();
                    set_PWM(TURN_SPEED, TURN_SPEED);
                }

                else if (last_error > 0) {
                    motors_yaw_right();
                    set_PWM(TURN_SPEED, TURN_SPEED);     
                }

                else {
                    motors_brake();
                    set_PWM(0, 0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}