#include <sys/param.h>
#include "driver/ledc.h"

#include "motor.h"
#include "io_config.h"

MOTOR_PERIF motor_boom = {
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_pin_a = MCPWM0A,
    .mcpwm_pin_b = MCPWM0B,
    .timer = MCPWM_TIMER_0,
    .gpio_a = MOTOR_BOOM_A,
    .gpio_b = MOTOR_BOOM_B,
};

MOTOR_PERIF motor_hook = {
    .mcpwm_unit = MCPWM_UNIT_1,
    .mcpwm_pin_a = MCPWM0A,
    .mcpwm_pin_b = MCPWM0B,
    .timer = MCPWM_TIMER_0,
    .gpio_a = MOTOR_HOOK_A,
    .gpio_b = MOTOR_HOOK_B,
};



void _init_single_motor(MOTOR_PERIF motor) {

    mcpwm_gpio_init(motor.mcpwm_unit, motor.mcpwm_pin_a, motor.gpio_a);
    mcpwm_gpio_init(motor.mcpwm_unit, motor.mcpwm_pin_b, motor.gpio_b);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 10000;                        // frequency = 1kHz,
    pwm_config.cmpr_a = 0;                              // initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                              // initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;         // up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(motor.mcpwm_unit, motor.timer, &pwm_config);    // Configure PWM0A & PWM0B with above settings
    mcpwm_start(motor.mcpwm_unit, motor.timer);

}


void setup_pwm(uint8_t pin, uint8_t channel) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_3,
        .duty_resolution  = LEDC_TIMER_12_BIT,
        .freq_hz          = 10000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = channel,
        .timer_sel      = LEDC_TIMER_3,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pin,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}


void init_motors() {
    _init_single_motor(motor_boom);
    _init_single_motor(motor_hook);
    setup_pwm(MOTOR_DRIVE_A, LEDC_CHANNEL_4);
    setup_pwm(MOTOR_DRIVE_B, LEDC_CHANNEL_5);
}


void set_motor(MOTOR_PERIF motor, float duty_cycle)
{
    /* motor moves in forward direction, with duty cycle = duty % */
    if (duty_cycle > 0) {
        mcpwm_set_signal_low(motor.mcpwm_unit, motor.timer, MCPWM_OPR_A);
        mcpwm_set_duty(motor.mcpwm_unit, motor.timer, MCPWM_OPR_B, duty_cycle / 10.0f);
        mcpwm_set_duty_type(motor.mcpwm_unit, motor.timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    /* motor moves in backward direction, with duty cycle = -duty % */
    else {
        mcpwm_set_signal_low(motor.mcpwm_unit, motor.timer, MCPWM_OPR_B);
        mcpwm_set_duty(motor.mcpwm_unit, motor.timer, MCPWM_OPR_A, -duty_cycle / 10.0f);
        mcpwm_set_duty_type(motor.mcpwm_unit, motor.timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
}

/*
    Value from -1000 to 1000
*/
void set_drive(int value) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, 4095 * MAX(0, value) / 1000);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5, 4095 * -MIN(0, value) / 1000);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5);
}
