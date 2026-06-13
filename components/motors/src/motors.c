#include "motors.h"
#include "board.h"
#include "driver/gpio.h"

void motors_control_init(void)
{
  gpio_set_direction(HB_AIN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(HB_AIN2, GPIO_MODE_OUTPUT);
  gpio_set_direction(HB_BIN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(HB_BIN2, GPIO_MODE_OUTPUT);
}

static void motor_left_forward(void)
{
  gpio_set_level(HB_AIN1,0);
  gpio_set_level(HB_AIN2,1);
}

static void motor_left_backward(void)
{
  gpio_set_level(HB_AIN1,1);
  gpio_set_level(HB_AIN2,0);
}

static void motor_left_coast(void)
{
  gpio_set_level(HB_AIN1,0);
  gpio_set_level(HB_AIN2,0);
}

static void motor_left_brake(void)
{
  gpio_set_level(HB_AIN1,1);
  gpio_set_level(HB_AIN2,1);
}

static void motor_right_forward(void)
{
  gpio_set_level(HB_BIN1,0);
  gpio_set_level(HB_BIN2,1);
}

static void motor_right_backward(void)
{
  gpio_set_level(HB_BIN1,1);
  gpio_set_level(HB_BIN2,0);
}

static void motor_right_coast(void)
{
  gpio_set_level(HB_BIN1,0);
  gpio_set_level(HB_BIN2,0);
}

static void motor_right_brake(void)
{
  gpio_set_level(HB_BIN1,1);
  gpio_set_level(HB_BIN2,1);
}

void motors_forward(void)
{
  motor_left_forward();
  motor_right_forward();
}

void motors_backward(void)
{
  motor_left_backward();
  motor_right_backward();
}

void motors_coast(void)
{
  motor_left_coast();
  motor_right_coast();
}

void motors_brake(void)
{
  motor_left_brake();
  motor_right_brake();
}

void motors_yaw_right(void)
{
  motor_left_forward();
  motor_right_backward();
}

void motors_yaw_left(void)
{
  motor_left_backward();
  motor_right_forward();
}
