#include "esp_log.h"
#include "math.h"

#include "joystick.h"
#include "io_config.h"
#include "util.h"

int prev_x = 0;
int prev_y = 0;

int center_x = 1190;
int center_y = 1260;

// square deadzone half side length
#define DEADZONE 40
#define THRESH 20


void setup_adc(adc1_channel_t adc_channel) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
}


bool poll_joystick(int *x, int *y) {
    int raw_x = adc1_get_raw(X_AXIS);
    int raw_y = adc1_get_raw(Y_AXIS);
    

    // hysteresis (since last poll)
    bool changed = (abs(prev_x - raw_x) > THRESH) || (abs(prev_y - raw_y) > THRESH);
    if (changed) {
        prev_x = raw_x;
        prev_y = raw_y;
    }


    // calibrate values
    int temp_x = -(raw_x - center_x) * (1000 + DEADZONE) / center_x;
    *x = (temp_x > 0) ? MAX(0, temp_x - DEADZONE) : MIN(0, temp_x + DEADZONE);
    int temp_y = (raw_y - center_y) * (1000 + DEADZONE) / center_y;
    *y = (temp_y > 0) ? MAX(0, temp_y - DEADZONE) : MIN(0, temp_y + DEADZONE);

    // printf("%4d\t%4d\t:\t%4d\t%4d \n", raw_x, raw_y, *x, *y);

    return changed;
}
