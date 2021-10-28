#include "motor.h"
#include "io_config.h"

MOTOR_PERIF motor_turret = {
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_pin_a = MCPWM0A,
    .mcpwm_pin_b = MCPWM0B,
    .timer = MCPWM_TIMER_0,
    .gpio_a = MOTOR_TURRET_A,
    .gpio_b = MOTOR_TURRET_B,
};

MOTOR_PERIF motor_boom = {
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_pin_a = MCPWM1A,
    .mcpwm_pin_b = MCPWM1B,
    .timer = MCPWM_TIMER_0,
    .gpio_a = MOTOR_BOOM_A,
    .gpio_b = MOTOR_BOOM_B,
};

MOTOR_PERIF motor_hook = {
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_pin_a = MCPWM2A,
    .mcpwm_pin_b = MCPWM2B,
    .timer = MCPWM_TIMER_0,
    .gpio_a = MOTOR_HOOK_A,
    .gpio_b = MOTOR_HOOK_B,
};


void _init_single_motor(MOTOR_PERIF motor) {

    mcpwm_gpio_init(motor.mcpwm_unit, motor.mcpwm_pin_a, motor.gpio_a);
    mcpwm_gpio_init(motor.mcpwm_unit, motor.mcpwm_pin_b, motor.gpio_b);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;                        // frequency = 1kHz,
    pwm_config.cmpr_a = 0;                              // initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                              // initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;         // up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(motor.mcpwm_unit, motor.timer, &pwm_config);    // Configure PWM0A & PWM0B with above settings
    mcpwm_start(motor.mcpwm_unit, motor.timer);

}

void init_motors() {
    _init_single_motor(motor_boom);
    _init_single_motor(motor_hook);
    _init_single_motor(motor_turret);
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
