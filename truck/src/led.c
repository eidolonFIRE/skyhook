#include <sys/param.h>

#include "led.h"
#include "io_config.h"


LED_t led_red = {LEDRED, LEDC_CHANNEL_1};
LED_t led_green = {LEDGREEN, LEDC_CHANNEL_2};
LED_t led_blue = {LEDBLUE, LEDC_CHANNEL_3};


void setup_led(LED_t led) {
    // gpio_pad_select_gpio(pin);
    // gpio_set_direction(pin, GPIO_MODE_OUTPUT);
 
    // LED 
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_12_BIT,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = led.pwm_channel,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led.gpio_pin,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void init_leds() {
    setup_led(led_red);
    setup_led(led_green);
    setup_led(led_blue);
}

void set_led_duty(LED_t led, u_int32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, led.pwm_channel, 4095 * duty / 1000);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, led.pwm_channel);
}


/* 
    Red, Green, Blue values on scale 0 - 1000 
*/
void led_color(uint32_t red, uint32_t green, uint32_t blue) {
    
    set_led_duty(led_red, red);
    set_led_duty(led_green, green);
    set_led_duty(led_blue, blue);
}

