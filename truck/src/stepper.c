#include "math.h"

#include "io_config.h"
#include "stepper.h"


void init_stepper() {

    // PWM stepper motor (led perif)
    ledc_timer_config_t ledc_timer_mtr = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 100,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_mtr));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_mtr = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_STEPPER_STEP,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_mtr));

    // setup IO for directional control
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << MOTOR_STEPPER_DIR,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
}


void set_stepper(int32_t value) {
    float curve = pow(abs(value) / 1000.0, 1.5);


    if (abs(value) > 0) {
        // enable pwm signal
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, (1<<4));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

        // set frequency
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, curve * 1000 + 10);
        gpio_set_level(MOTOR_STEPPER_DIR, value > 0);
    } else {
        // disable pwm signal
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    }
}
