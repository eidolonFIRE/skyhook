#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "include/rotary_encoder.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include <sys/param.h>

// Project files
#include "io_config.h"
#include "wifi_config.h"
#include "led.h"
#include "joystick.h"



#define TAG "app ===== "


#define ENABLE_HALF_STEPS false  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0      // Set to a positive non-zero number to reset the position if this value is exceeded

static xQueueHandle gpio_evt_queue = NULL;

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_UPDATE BIT3



typedef struct {
    int32_t drive;
    int32_t steering;
    int32_t turret;
    int32_t boom;
    int32_t hook;
} CONTROL_t;

CONTROL_t control_msg = {0,0,0,0,0};

int joystick_x = 0;
int joystick_y = 0;
int wheel = 0;
int wheel_prev = 0;

bool flag_update_msg = false;




static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    int batt_counter = 0;

    while (1) {

        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(HOST_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        led_color(0, 300, 0);

        while (1) {
            flag_update_msg = false;
            while (!flag_update_msg) {
                vTaskDelay(20 / portTICK_PERIOD_MS);
                if (poll_joystick(&joystick_x, &joystick_y)) break;
            }

            if (gpio_get_level(MODE_SWITCH)) {
                // driving mode
                led_color(0, 300, 500);
                control_msg.drive = joystick_y;
                control_msg.steering = wheel - wheel_prev;
                control_msg.turret = 0;
                control_msg.boom = 0;
                control_msg.hook = 0;
            } else {
                // turret mode
                led_color(0, 300, 0);
                control_msg.drive = 0;
                control_msg.steering = 0;
                control_msg.turret = joystick_x;
                control_msg.boom = joystick_y;
                control_msg.hook = wheel - wheel_prev;
            }
            wheel_prev = wheel;
            

            int err = sendto(sock, &control_msg, sizeof(control_msg), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                break;
            }

            // struct sockaddr_in sourceAddr;
            // socklen_t socklen = sizeof(sourceAddr);
            // int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

            // // Error occured during receiving
            // if (len < 0) {
            //     ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            //     break;
            // }
            // // Data received
            // else {
            //     rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            //     ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            //     ESP_LOGI(TAG, "%s", rx_buffer);
            // }

            // check battery every so often
            
        }

        led_color(800, 0, 0);

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}



static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}



void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            // .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}


static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // flag_update_msg = true;
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            flag_update_msg = true;
            // printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}


void app_main()
{
    init_leds();

    // initial state
    led_color(700, 600, 0);

    setup_adc(X_AXIS);
    setup_adc(Y_AXIS);

    setup_adc(BATT_VOL);

    // setup mode switch
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<MODE_SWITCH),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // GPIO interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MODE_SWITCH, gpio_isr_handler, (void*)MODE_SWITCH);


    //Initialize NVS / wifi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_sta();



    // init rotary encoder
    // ESP_ERROR_CHECK(gpio_install_isr_service(0));
    rotary_encoder_info_t info = { 0 };
    ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
    // ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
    // ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));

    // Create a queue for events from the rotary encoder driver.
    // Tasks can read from this queue to receive up to date position information.
    QueueHandle_t event_queue = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));

    int batt_counter = 0;
    while (1) {
        // Wait for incoming events on the event queue.
        rotary_encoder_event_t event = { 0 };
        if (xQueueReceive(event_queue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            ESP_LOGI(TAG, "Event: position %d, direction %s", event.state.position,
                     event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
            wheel = event.state.position;
            flag_update_msg = true;
            if (batt_counter-- < 0) {
                batt_counter = 100;
                float batt_v = adc1_get_raw(BATT_VOL) * 3.9 / 1420.0;
                ESP_LOGI(TAG, "Battery Voltage: %1.2fv", batt_v);
            }
        } else {
            // // Poll current position and direction
            // rotary_encoder_state_t state = { 0 };
            // ESP_ERROR_CHECK(rotary_encoder_get_state(&info, &state));
            // ESP_LOGI(TAG, "Poll: position %d, direction %s", state.position,
            //          state.direction ? (state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
            // // gpio_set_level(LEDGREEN, event.state.position % 2);
            // // gpio_set_level(LEDYELLOW, (event.state.position / 2) % 2);
            // // gpio_set_level(LEDRED, (event.state.position / 4) % 2);
            // // Reset the device
            // if (RESET_AT && (state.position >= RESET_AT || state.position <= -RESET_AT)) {
            //     ESP_LOGI(TAG, "Reset");
            //     ESP_ERROR_CHECK(rotary_encoder_reset(&info));
            // }
        }
    }

    


    ESP_LOGE(TAG, "queue receive failed");

    ESP_ERROR_CHECK(rotary_encoder_uninit(&info));
}

