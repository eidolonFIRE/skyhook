#include "driver/ledc.h"


typedef struct {
    u_int8_t gpio_pin;
    u_int8_t pwm_channel;
} LED_t;


void init_leds();
void led_color(uint32_t red, uint32_t green, uint32_t blue);
