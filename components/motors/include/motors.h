#pragma once
#include "driver/gpio.h"

void motors_control_init(void);
void motors_forward(void);
void motors_backward(void);
void motors_coast(void);
void motors_brake(void);
void motors_yaw_right(void);
void motors_yaw_left(void);
