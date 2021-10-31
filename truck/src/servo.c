#include "driver/ledc.h"

#include "io_config.h"
#include "servo.h"

void init_servo() {
    // LED (servo)
    ledc_timer_config_t ledc_timer_servo = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_TIMER_12_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_servo));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_servo = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_STEERING,
        .duty           = 240,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_servo));
}

void set_servo(uint32_t value) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, value);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}
