#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include <stdio.h>
#include "driver/gpio.h"
#include <driver/adc.h>
#include "sdkconfig.h"

#include "include/rotary_encoder.h"

#define TAG "app"

#define ROT_ENC_A_GPIO 26
#define ROT_ENC_B_GPIO 27

#define ENABLE_HALF_STEPS false  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0      // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION    false  // Set to true to reverse the clockwise/counterclockwise sense

#define LEDGREEN 25
#define LEDYELLOW 33
#define LEDRED 32


void setup_led(u_int8_t pin) {
    gpio_pad_select_gpio(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

void setup_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
}


void app_main()
{
    setup_led(LEDGREEN);
    setup_led(LEDYELLOW);
    setup_led(LEDRED);

    setup_adc();

// while(1) {
//     /* Blink off (output low) */
//     printf("Turning off the LED\n");
//     gpio_set_level(LEDGREEN, 0);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     /* Blink on (output high) */
//     printf("Turning on the LED\n");
//     gpio_set_level(LEDGREEN, 1);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
// }

    // esp32-rotary-encoder requires that the GPIO ISR service is installed before calling rotary_encoder_register()
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Initialise the rotary encoder device with the GPIOs for A and B signals
    rotary_encoder_info_t info = { 0 };
    ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
    // ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
#ifdef FLIP_DIRECTION
    ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
#endif

    // Create a queue for events from the rotary encoder driver.
    // Tasks can read from this queue to receive up to date position information.
    QueueHandle_t event_queue = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));

    while (1)
    {
        // Wait for incoming events on the event queue.
        rotary_encoder_event_t event = { 0 };
        if (xQueueReceive(event_queue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE)
        {
            ESP_LOGI(TAG, "Event: position %d, direction %s", event.state.position,
                     event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
            gpio_set_level(LEDGREEN, event.state.position % 2);
            gpio_set_level(LEDYELLOW, (event.state.position / 2) % 2);
            gpio_set_level(LEDRED, (event.state.position / 4) % 2);
        }
        else
        {
            // Poll current position and direction
            rotary_encoder_state_t state = { 0 };
            ESP_ERROR_CHECK(rotary_encoder_get_state(&info, &state));
            ESP_LOGI(TAG, "Poll: position %d, direction %s", state.position,
                     state.direction ? (state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
            // gpio_set_level(LEDGREEN, event.state.position % 2);
            // gpio_set_level(LEDYELLOW, (event.state.position / 2) % 2);
            // gpio_set_level(LEDRED, (event.state.position / 4) % 2);
            // Reset the device
            if (RESET_AT && (state.position >= RESET_AT || state.position <= -RESET_AT))
            {
                ESP_LOGI(TAG, "Reset");
                ESP_ERROR_CHECK(rotary_encoder_reset(&info));
            }
        }

        int val1 = adc1_get_raw(ADC1_CHANNEL_6);
        int val2 = adc1_get_raw(ADC1_CHANNEL_7);
        printf("Raw1: %d\tRaw2: %d  \n", val1, val2);
    }

    


    ESP_LOGE(TAG, "queue receive failed");

    ESP_ERROR_CHECK(rotary_encoder_uninit(&info));
}

