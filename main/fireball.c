#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

      if (b_le && b_li && b_ri && b_re)
      {
        finish_counter++;
        printf("DO NOTHING... %d    ", finish_counter);
        if (finish_counter >= FINISH_TIME)
        {
          motors_brake();
          set_PWM(0, 0);
          finished = true;
          printf("Ended!");
        }
      }
      else
      {
        finish_counter = 0;

        if (b_le) {
          motors_yaw_left();
          set_PWM(TURN_SPEED, TURN_SPEED);
          last_error = -2;
          printf("HARD TURN LEFT   ");
        }
        else if (b_re)
        {
          motors_yaw_right();
          set_PWM(TURN_SPEED, TURN_SPEED);
          last_error = 2;
          printf("HARD TURN RIGHT   ");
        } else if (b_li && !b_ri) {
                motors_forward();
                set_PWM(MED_SPEED, MAX_SPEED);
                last_error = -1;
                printf("SMALL TURN LEFT   ");
            }

            else if (!b_li && b_ri) {
                motors_forward();
                set_PWM(MAX_SPEED, MED_SPEED);
                last_error = 1;
                printf("SMALL TURN RIGHT   ");
            }

            else if (b_li && b_ri) {
                motors_forward();
                set_PWM(MAX_SPEED, MAX_SPEED);
                printf("STRAIGHT   ");
            }

            else {
                if (last_error < 0) {
                    motors_yaw_left();
                    set_PWM(TURN_SPEED, TURN_SPEED);
                    printf("SEARCHING LEFT   ");
                }

                else if (last_error > 0) {
                    motors_yaw_right();
                    set_PWM(TURN_SPEED, TURN_SPEED);
                    printf("SEARCHING RIGHT   ");     
                }

                else {
                    motors_brake();
                    set_PWM(0, 0);
                    printf("NOTHING TO SEARCH   ");
                }
            }
        }
      
      vTaskDelay(pdMS_TO_TICKS(100));

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

    xTaskCreate(
        drive_task,
        "drive_task",
        4096,
        NULL,
        5,
        NULL
    );
}
