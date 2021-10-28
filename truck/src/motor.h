#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

typedef struct {
    mcpwm_unit_t mcpwm_unit;
    mcpwm_timer_t timer;
    u_int32_t gpio_a;
    u_int32_t gpio_b;
    mcpwm_io_signals_t mcpwm_pin_a;
    mcpwm_io_signals_t mcpwm_pin_b;
} MOTOR_PERIF;

MOTOR_PERIF motor_drive;
MOTOR_PERIF motor_boom;
MOTOR_PERIF motor_hook;

void init_motors();
void set_motor(MOTOR_PERIF motor, float duty_cycle);
