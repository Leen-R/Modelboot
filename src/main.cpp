#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/ledc.h"
#include "esp_mac.h"

#define SERVO_PIN GPIO_NUM_4

typedef struct struct_message {
    int16_t steering;
    int16_t throttle;
} struct_message;

struct_message myData;

// Global variable to safely pass the steering value from the callback to the main loop
volatile int16_t target_steering = 0;

extern "C" void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    target_steering = myData.steering;
}

extern "C" void app_main(void) {

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    // 1. Initialize Wi-Fi and ESP-NOW
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv);

    // 2. Configure the LEDC peripheral to generate a 50Hz Servo PWM signal
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution  = LEDC_TIMER_14_BIT; // 16384 total steps
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.freq_hz          = 50;                // 50Hz = 20ms period (standard RC)
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num       = SERVO_PIN;
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = LEDC_CHANNEL_0;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);

    // // Fetch and print the MAC address
    // uint8_t mac[6];
    // esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    // printf("\n=========================================\n");
    // printf("DEVICE MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\n", 
    //        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // printf("=========================================\n\n");

    while (1) {
        // Map the -100 to 100 steering value to a 1000us to 2000us pulse
        // 0 steering = 1500us (center)
        uint32_t pulse_us = 1500 + (target_steering * 5); 

        // Convert the microsecond pulse length into a 14-bit duty cycle fraction
        // Formula: (pulse_length_us / 20000us) * 16384 steps
        uint32_t duty = (pulse_us * 16384) / 20000;

        // Apply the PWM signal to the servo
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        printf("Steering Payload: %d | Servo Pulse: %lu us\n", target_steering, pulse_us);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}