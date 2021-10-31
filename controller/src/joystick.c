#include "esp_log.h"
#include "math.h"

#include "joystick.h"
#include "io_config.h"
#include "util.h"

int prev_x = 0;
int prev_y = 0;

int center_x = 1200;
int center_y = 1400;

// square deadzone half side length
#define DEADZONE_X 50
#define DEADZONE_Y 40
#define THRESH 5


void setup_adc(adc1_channel_t adc_channel) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
}


bool poll_joystick(int *x, int *y) {
    int raw_x = adc1_get_raw(X_AXIS);
    int raw_y = adc1_get_raw(Y_AXIS);
    




    // calibrate values
    int temp_x = -(raw_x - center_x) * (1000 + DEADZONE_X) / center_x;
    *x = (temp_x > 0) ? MAX(0, temp_x - DEADZONE_X) : MIN(0, temp_x + DEADZONE_X);
    int temp_y = (raw_y - center_y) * (1000 + DEADZONE_Y) / center_y;
    *y = (temp_y > 0) ? MAX(0, temp_y - DEADZONE_Y) : MIN(0, temp_y + DEADZONE_Y);

    // printf("%4d\t%4d\t:\t%4d\t%4d \n", raw_x, raw_y, *x, *y);

    // hysteresis (since last poll)
    bool changed = (abs(prev_x - *x) > THRESH) || (abs(prev_y - *y) > THRESH);
    if (changed) {
        prev_x = *x;
        prev_y = *y;
    }

    return changed;
}
