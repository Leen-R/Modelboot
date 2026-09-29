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
#define THROTTLE_PIN GPIO_NUM_5 // <-- Added pin for the throttle (ESC)

typedef struct struct_message {
    int16_t steering;
    int16_t throttle;
} struct_message;

struct_message myData;

// Global variables
volatile int16_t target_steering = 0;
volatile int16_t target_throttle = 0;

extern "C" void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    target_steering = myData.steering;
    target_throttle = myData.throttle;
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

    // 2. Configure the shared timer (50Hz for standard RC)
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution  = LEDC_TIMER_14_BIT; // 16384 total steps
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.freq_hz          = 50;                
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // 3. Configure Steering Channel (Channel 0 on GPIO 4)
    ledc_channel_config_t steering_channel = {};
    steering_channel.gpio_num       = SERVO_PIN;
    steering_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    steering_channel.channel        = LEDC_CHANNEL_0;
    steering_channel.intr_type      = LEDC_INTR_DISABLE;
    steering_channel.timer_sel      = LEDC_TIMER_0;
    steering_channel.duty           = 0;
    steering_channel.hpoint         = 0;
    ledc_channel_config(&steering_channel);

    // 4. Configure Throttle Channel (Channel 1 on GPIO 5)
    ledc_channel_config_t throttle_channel = {};
    throttle_channel.gpio_num       = THROTTLE_PIN;
    throttle_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    throttle_channel.channel        = LEDC_CHANNEL_1; // Must be a different channel
    throttle_channel.intr_type      = LEDC_INTR_DISABLE;
    throttle_channel.timer_sel      = LEDC_TIMER_0;   // Shares the same timer
    throttle_channel.duty           = 0;
    throttle_channel.hpoint         = 0;
    ledc_channel_config(&throttle_channel);

    while (1) {
        // Map the payload values to a 1000us to 2000us pulse
        uint32_t steering_pulse_us = 1500 + (target_steering * 5); 
        uint32_t throttle_pulse_us = 1500 + (target_throttle * 5); 

        // Convert the microsecond pulse lengths into a 14-bit duty cycle fraction
        uint32_t steering_duty = (steering_pulse_us * 16384) / 20000;
        uint32_t throttle_duty = (throttle_pulse_us * 16384) / 20000;

        // Apply PWM to Steering (Channel 0)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, steering_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        // Apply PWM to Throttle (Channel 1)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, throttle_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

        printf("Steering: %d -> %lu us | Throttle: %d -> %lu us\n", 
                target_steering, steering_pulse_us, target_throttle, throttle_pulse_us);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}