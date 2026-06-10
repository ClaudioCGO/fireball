#include "board.h"
#include "driver/gpio.h"

/* H-bridge control gpios */
const gpio_num_t HB_AIN2 = GPIO_NUM_6;
const gpio_num_t HB_AIN1 = GPIO_NUM_7;
const gpio_num_t HB_BIN1 = GPIO_NUM_10;
const gpio_num_t HB_BIN2 = GPIO_NUM_20;

/* H-bridge pwm pins */
const gpio_num_t HB_PWMA = GPIO_NUM_5;
const gpio_num_t HB_PWMB = GPIO_NUM_21;

/* Infrared sensors pins */
const gpio_num_t IR_L_OUT = GPIO_NUM_4;
const gpio_num_t IR_L_IN = GPIO_NUM_3;
const gpio_num_t IR_R_IN = GPIO_NUM_1;
const gpio_num_t IR_R_OUT = GPIO_NUM_0;
