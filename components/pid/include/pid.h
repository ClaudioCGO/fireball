#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd);

float pid_compute(pid_controller_t *pid, float current_error);

void pid_reset(pid_controller_t *pid);