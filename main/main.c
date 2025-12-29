/*
 * main.c
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"

#include "led_strip.h"
#include "WRCDefs.h"
#include "myNvs.h"
#include "myTcm.h"
#include "myUsb.h"

#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>


#define TAG "WRC"

// ----------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------
static led_strip_handle_t led_strip;
static bool tcmProcessOk = true;
static int loopCounter = 0;

// USB selection
static bool useTCM = false;

// Log selection
/*
* Levels available:
    •	ESP_LOG_NONE
    •	ESP_LOG_ERROR
    •	ESP_LOG_WARN
    •	ESP_LOG_INFO
    •	ESP_LOG_DEBUG
    •	ESP_LOG_VERBOSE
    hint: Run idf.py menuconfig, can set the default log level
*/
static int logLevel = ESP_LOG_INFO;

// Plot selection
static int serial_plot = 1;

// ----------------------------------------------------------------------
// LED Helpers
// ----------------------------------------------------------------------
static void setPixelColor(int idx, uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, idx, r, g, b);
    led_strip_refresh(led_strip);
}

static void initLED(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_PIN,
        .max_leds = NUM_LEDS,
        .led_model = LED_MODEL_SK6812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000 // 10 MHz
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    //ESP_LOGI(TAG, "LED strip initialized with %d LEDs on GPIO %d", NUM_LEDS, RGB_PIN);
}

static void sequenceLED(void) {
    // Non-blocking startup indicator
    for (int color = 0; color < 3; ++color) {
        for (int i = 0; i < NUM_LEDS; i++) {
            uint8_t r = (color == 0) ? 255 : 0;
            uint8_t g = (color == 1) ? 255 : 0;
            uint8_t b = (color == 2) ? 255 : 0;
            setPixelColor(i, r, g, b);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    // Turn off
    for (int i = 0; i < NUM_LEDS; i++)
        setPixelColor(i, 0, 0, 0);
}

static void runLED(void) {
    static uint32_t lastToggle = 0;
    static bool ledState = false;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (now - lastToggle >= LED_TIME_MS) {
        lastToggle = now;
        ledState = !ledState;

        for (int i = 0; i < NUM_LEDS; i++) {
            if (!tcmProcessOk) {
                setPixelColor(i, 100, 0, 0); // TCM error = red
            }
            else if (ledState) {
                setPixelColor(i, 0, 100, 0); // green
            }
            else {
                setPixelColor(i, 0, 0, 0);   // off
            }
        }
    }
}

// ----------------------------------------------------------------------
// Application Entry Point
// ----------------------------------------------------------------------
void app_main(void) {

	init_rs485();

    esp_log_level_set("*", logLevel);
    esp_log_level_set(TAG, logLevel);

    ESP_LOGI(TAG, "==============================================================");
    ESP_LOGI(TAG, "  WRC System Startup");
    ESP_LOGI(TAG, "  Build date: " __DATE__ " " __TIME__);
	ESP_LOGI(TAG, "  Use TCM: %s", useTCM ? "true" : "false");   
    ESP_LOGI(TAG, "  Log Level: %d", logLevel);
	ESP_LOGI(TAG, "  Serial Plot: %d", serial_plot);    

#if defined(CONFIG_IDF_TARGET_ESP32S2)
    //ESP_LOGI(TAG, "  Target: ESP32-S2");
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    //SP_LOGI(TAG, "  Target: ESP32-S3");
#else
    //ESP_LOGI(TAG, "  Target: Unknown ESP32 variant");
#endif

    ESP_LOGI(TAG, "==============================================================");
    
    // Initialize peripherals
    initLED();
    sequenceLED();

    if (useTCM) {

        if (!initUsb(logLevel)) {
            ESP_LOGE(TAG, "USB initialization failed");
            send_rs485("ERR:USB_INIT");

            for (;;) {
                setPixelColor(0, 0, 0, 255);
                vTaskDelay(pdMS_TO_TICKS(200));
                setPixelColor(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        ESP_LOGI(TAG, "USB initialized");

        if (!initTcm(logLevel)) {
            ESP_LOGE(TAG, "TCM initialization failed");
            send_rs485("ERR:TCM_INIT");

            for (;;) {
                setPixelColor(0, 255, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                setPixelColor(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        ESP_LOGI(TAG, "TCM initialized");
    }

    ESP_LOGI(TAG, "Initialization complete. Entering main loop...");

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    while (1) {
        runLED();

        ESP_LOGI(TAG, "Loop %d", ++loopCounter);
        char uartMessage[50];
        sprintf(uartMessage, "Uart Loop %d", loopCounter);
        send_rs485(uartMessage);

        if (useTCM) {
            tcmProcessOk = runTcm(serial_plot);
        }

        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_RATE_MS));
    }
}
