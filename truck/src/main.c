#include <stdio.h>

#include <driver/adc.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>


// Project Files
#include "wifi_config.h"
#include "io_config.h"
#include "motor.h"
#include "led.h"

#define TAG "app ===== "

static EventGroupHandle_t s_wifi_event_group;

typedef struct {
    int32_t value;
    int32_t min;
    int32_t rate;
    int32_t max;
} CONTROL_t;

typedef struct {
    int32_t value;
    int32_t target;
    int32_t rate;
    int32_t max_rate;
} CONTROL_RATE_t;

CONTROL_t control_drive     = {  0, -1000,  50, 1000};
CONTROL_t control_steering  = {240,   180,   0,  300}; // servo
CONTROL_t control_turret    = {  0, -1500,  20, 1500}; // stepper
CONTROL_t control_boom      = {  0, -1000, 100, 1000};
CONTROL_RATE_t control_hook = {  0,     0,   0, 1000};

struct {
    int32_t drive;
    int32_t steering;
    int32_t turret;
    int32_t boom;
    int32_t hook;
} *rx_control_msg;

char rx_buffer[1024];

void clamp_control(CONTROL_t *control) {
    control->value = MIN(control->max, MAX(control->min, control->value));
}

void slew_control(CONTROL_t *control, int32_t value) {
    control->value += MIN(control->rate, MAX(-control->rate, (value - control->value)));
    clamp_control(control);
}

void add_control(CONTROL_t *control, int32_t value) {
    control->value += value;
    clamp_control(control);
}

void rate_control(CONTROL_RATE_t *control) {
    // integrate
    control->value += control->rate;

    // update velocity
    int32_t target_rate = (control->target - control->rate) / 10;
    control->rate = MIN(control->max_rate, MAX(-control->max_rate, target_rate));
}


void init_adc(adc1_channel_t adc_channel) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
}

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







static void udp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);


        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket binded");

        while (1) {

            // ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 sourceAddr;
            socklen_t socklen = sizeof(sourceAddr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&sourceAddr, &socklen);

            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (sourceAddr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (sourceAddr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                // Apply here because it's a delta value
                add_control(&control_steering, rx_control_msg->steering);
                control_hook.target += rx_control_msg->hook * 100;

                // DEBUG: control values coming from controller
                // ESP_LOGI(TAG, "Dr %4d, St %2d, Bm %4d, Tr %4d, Hk %2d\n", rx_control_msg->drive, rx_control_msg->steering, rx_control_msg->boom, rx_control_msg->turret, rx_control_msg->hook);

                // int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&sourceAddr, sizeof(sourceAddr));
                // if (err < 0) {
                //     ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                //     break;
                // }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}



static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        led_color(0, 300, 0);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        led_color(800, 600, 0);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void wifi_init_softap()
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);

    // start server
    xTaskCreate(udp_server_task, "udp_server", 4096 * 2, NULL, 5, NULL);

}


static void check_battery() {
    float batt_v = 0;
    while (true) {
        batt_v = adc1_get_raw(BATT_VOL) * 11.4 / 1075.0;
        ESP_LOGI(TAG, "Battery Voltage: %1.2fv", batt_v);
        if (batt_v < (3.8 * 4.0)) {
            // Low Battery Warning
            led_color(800, 0, 0);
        }
        // seconds * 100
        vTaskDelay(120 * 100);
    }
}


void app_main() {

    // Setup Perifs
    init_adc(BATT_VOL);
    init_leds();
    init_servo();
    init_stepper();
    init_motors();

    // Battery checker
    xTaskCreate(check_battery, "bat_checker", 2048, NULL, tskIDLE_PRIORITY, NULL);

    // initial state
    led_color(700, 600, 0);

    // Setup Wifi / Server
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_softap();

    rx_control_msg = rx_buffer;

    while (1) {
        // ESP_LOGI(TAG, "Dr %4d, St %2d, Bm %4d, Tr %4d, Hk %2d\n", control_drive.value, control_steering.value, control_boom.value, control_turret.value, control_hook.target);

        // Slew controls
        slew_control(&control_drive, rx_control_msg->drive);
        slew_control(&control_boom, rx_control_msg->boom);
        slew_control(&control_turret, rx_control_msg->turret);
        rate_control(&control_hook);

        // Update Motor Controls
        set_motor(motor_drive, -control_drive.value);
        set_motor(motor_boom, control_boom.value);
        set_motor(motor_hook, control_hook.rate);

        // (stepper)
        if (abs(control_turret.value) > 0) {
            // enable pwm signal
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, (1<<4));
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

            // set frequency
            ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, abs(control_turret.value) + 10);
            gpio_set_level(MOTOR_STEPPER_DIR, control_boom.value < 0);
        } else {
            // disable pwm signal
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        }

        // (servo)
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, control_steering.value);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

        // Controls refresh rate
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}