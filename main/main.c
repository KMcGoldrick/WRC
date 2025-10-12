#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "WRCDefs.h"
#include "nvs_flash.h"
#include "myNvs.h"
#include "myTcm.h"
#include "myUsb.h"
#include "nvs.h"
#include <stdio.h>

int loopCounter = 0;
bool erase_nvs = false;
bool processRunning = false;
led_strip_handle_t led_strip;

unsigned long millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void setPixelColor(int idx, uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, idx, r, g, b);
    led_strip_refresh(led_strip);
}

void initLED() {
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_PIN,
        .max_leds = NUM_LEDS,
        .led_model = LED_MODEL_SK6812, // or LED_MODEL_WS2812
        .flags.invert_out = false
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10000000
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

void sequenceLED() {
    for (int i = 0; i < NUM_LEDS; i++) {
        setPixelColor(i, 255, 0, 0); // Red
        vTaskDelay(pdMS_TO_TICKS(100));
        setPixelColor(i, 0, 0, 0); // Off
    }
    for (int i = 0; i < NUM_LEDS; i++) {
        setPixelColor(i, 0, 255, 0); // Green
        vTaskDelay(pdMS_TO_TICKS(100));
        setPixelColor(i, 0, 0, 0); // Off
    }
    for (int i = 0; i < NUM_LEDS; i++) {
        setPixelColor(i, 0, 0, 255); // Blue
        vTaskDelay(pdMS_TO_TICKS(100));
        setPixelColor(i, 0, 0, 0); // Off
    }
}

void runLED() {
    static unsigned long lastToggleTime = 0; // Tracks the last time the LED state was toggled
    unsigned long currentTime = millis();
    int ledTime_ms = 1000;

    if (processRunning) {
        ledTime_ms = LEDTime_ms;
    }
    else {
        ledTime_ms = LEDTime_ms / 2;
    }

    if (currentTime - lastToggleTime >= ledTime_ms) { // Check if interval has elapsed
        lastToggleTime = currentTime; // Update the last toggle time
        static bool ledState = false; // Tracks the current state of the LED

        ledState = !ledState; // Toggle the LED state

        if (ledState) {
            // Turn LED on (e.g., green)
            for (int i = 0; i < NUM_LEDS; i++) {
                setPixelColor(i, 0, 1, 0); // Green
            }
        } else {
            // Turn LED off
            for (int i = 0; i < NUM_LEDS; i++) {
                setPixelColor(i, 0, 0, 0); // Off
            }
		}
    }
}

void app_main(void)
{
#ifdef CONFIG_IDF_TARGET_ESP32S2
    ESP_LOGI("WRC", "Hello from the ESP32-S2!......................................");
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
	ESP_LOGI("WRC", "Hello from the ESP32-S3!......................................");
#endif
	initLED();
	sequenceLED();
	initNvsLog(erase_nvs);
    initUsb();
    //initTcm();


    while (1) {
        runLED();
        //runTcm();

        ESP_LOGI("WRC", "Looping... %d", ++loopCounter);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
