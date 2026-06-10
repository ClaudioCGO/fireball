#include "pwm.h"
#include "driver/ledc.h"

#define SPEED_MODE LEDC_LOW_SPEED_MODE // C3 limitation, means cant change timer settings on the fly
#define TIMER_PWM LEDC_TIMER_0 // Both use the same timer

#define CHANNEL_LEFT LEDC_CHANNEL_0
#define CHANNEL_RIGHT LEDC_CHANNEL_1

static int _pwm_left = 0;
static int _pwm_right = 0;


void configure_pwm(pwm_config_t pin_config) {
    //Define the pins
    _pwm_left = pin_config.left_pin;
    _pwm_right = pin_config.right_pin;

    // First configure timer
    ledc_timer_config_t pwm_timer = {
        .speed_mode = SPEED_MODE,
        .timer_num  = TIMER_PWM,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 5000, // This frequency supports until 13 bits
        .clk_cfg = LEDC_AUTO_CLK
    }; 
    
    ledc_timer_config(&pwm_timer);

    // Then configure the channels

    // Left channel
    ledc_channel_config_t left_channel = {
        .gpio_num = _pwm_left,
        .speed_mode = SPEED_MODE,
        .channel = CHANNEL_LEFT,
        .timer_sel = TIMER_PWM,
        .duty = 0,
        .hpoint = 0 // delay to start the pulse in the cycle, useful to prevent energy spikes
    };

    ledc_channel_config(&left_channel);

    // Right channel
    ledc_channel_config_t right_channel = {
        .gpio_num = _pwm_right,
        .speed_mode = SPEED_MODE,
        .channel = CHANNEL_RIGHT,
        .timer_sel = TIMER_PWM,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&right_channel);
}

void set_PWM(uint32_t left_speed, uint32_t right_speed) {

    // Limit to the resolution max - 1 (0 to 4095)
    if (left_speed > 4095) left_speed = 4095;
    if (right_speed > 4095) right_speed = 4095;

    // Change left PWM
    ledc_set_duty(SPEED_MODE, CHANNEL_LEFT, left_speed); // Not thread safe because it depends on 2 commands
    ledc_update_duty(SPEED_MODE, CHANNEL_LEFT); // Changes the duty only in the next cycle

    // Change right PWM
    ledc_set_duty(SPEED_MODE, CHANNEL_RIGHT, right_speed);
    ledc_update_duty(SPEED_MODE, CHANNEL_RIGHT);
}