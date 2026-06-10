#pragma once
#include "driver/ledc.h"

typedef struct {
    int left_pin;
    int right_pin;
} pwm_config_t;

void configure_pwm(pwm_config_t pin_config);
void set_PWM(uint32_t left_speed, uint32_t right_speed);