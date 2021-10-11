#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/pcnt.h"



#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B



#define MOTOR_CTRL_MCPWM_UNIT   MCPWM_UNIT_0
#define MOTOR_CTRL_MCPWM_TIMER  MCPWM_TIMER_0



void init() {
    mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0B, GPIO_PWM0B_OUT);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;     //frequency = 1kHz,
    pwm_config.cmpr_a = 0;                              //initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                              //initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;         //up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}


void brushed_motor_set_duty(float duty_cycle)
{
    /* motor moves in forward direction, with duty cycle = duty % */
    if (duty_cycle > 0) {
        mcpwm_set_signal_low(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
    }
    /* motor moves in backward direction, with duty cycle = -duty % */
    else {
        mcpwm_set_signal_low(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, -duty_cycle);
        mcpwm_set_duty_type(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
}

void app_main() {
    init();

    float duty = 0.0;
    while (1) {
        duty += 0.01;
        if (duty > 1) {
            duty = 0;
        }
        brushed_motor_set_duty(duty);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}