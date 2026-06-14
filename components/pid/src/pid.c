#include "pid.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd) {
    pid -> kp = kp;
    pid -> ki = ki;
    pid -> kd = kd;
    pid -> prev_error = 0.0f;
    pid -> integral = 0.0f;
}


float pid_compute(pid_controller_t *pid, float current_error) {
    float p = (pid -> kp) * current_error;

    pid -> integral += current_error;
    float i = (pid -> ki) * (pid ->integral);

    float derivative = current_error - (pid -> prev_error);
    float d = (pid -> kd) * derivative;

    pid -> prev_error = current_error;

    return p + i + d;
}


void pid_reset(pid_controller_t *pid) {
    pid -> prev_error = 0.0f;
    pid -> integral = 0.0f;
}