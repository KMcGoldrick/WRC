/*
 * main.c
 *
 * WRC system main application
 * Supports:
 *  - Dual USB (TCM + Logging)
 *  - LED strip status
 *  - PSRAM info display
 *  - Configurable NVS log and startup behavior
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

#define TAG "WRC"

 // ----------------------------------------------------------------------
 // Globals
 // ----------------------------------------------------------------------
static led_strip_handle_t led_strip;
static bool tcmProcessOk = true;
static int loopCounter = 0;

// USB selection
static bool useTCMUsb = false;
static bool useLogUsb = false;

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
    ESP_LOGI(TAG, "LED strip initialized with %d LEDs on GPIO %d", NUM_LEDS, RGB_PIN);
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
    esp_rom_printf(">>> APP MAIN STARTED <<<\n");

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "==============================================================");
    ESP_LOGI(TAG, "  WRC System Startup");
    ESP_LOGI(TAG, "  Build date: " __DATE__ " " __TIME__);

#if defined(CONFIG_IDF_TARGET_ESP32S2)
    ESP_LOGI(TAG, "  Target: ESP32-S2");
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGI(TAG, "  Target: ESP32-S3");
#else
    ESP_LOGI(TAG, "  Target: Unknown ESP32 variant");
#endif

    ESP_LOGI(TAG, "  PSRAM available: %d bytes", esp_psram_get_size());
    ESP_LOGI(TAG, "  Free PSRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "==============================================================");

    // Initialize peripherals
    initLED();
    sequenceLED();

    // Initialize dual USB (TCM + Logging)
    if (useTCMUsb || useLogUsb) {
        if (!initUsbDual()) {
            ESP_LOGE(TAG, "USB initialization failed");
            while (1) {
                setPixelColor(0, 255, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                setPixelColor(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
    }

    if (useTCMUsb) {
        ESP_LOGI(TAG, "Waiting for TCM USB device...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!connectDevice(TCM_VID, TCM_PID)) {
            ESP_LOGE(TAG, "TCM device connection failed");
            tcmProcessOk = false;
        }
    }

    ESP_LOGI(TAG, "Initialization complete. Entering main loop...");

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    while (1) {
        runLED();

        ESP_LOGI(TAG, "Loop %d", ++loopCounter);

        if (useTCMUsb) {
            // Fetch TCM sensor data
            rawSensors sensor;
            if (getSensorsRawUSB(true, &sensor, "GSR")) {
                tcmProcessOk = true;
                ESP_LOGI(TAG, "TCM Temp=%u Acc=(%d,%d,%d) Mag=(%d,%d,%d) Batt=%u",
                    sensor.temp, sensor.acc.x, sensor.acc.y, sensor.acc.z,
                    sensor.mag.x, sensor.mag.y, sensor.mag.z, sensor.batt);
            }
            else {
                tcmProcessOk = false;
                ESP_LOGW(TAG, "TCM read failed");
            }
        }

        if (useLogUsb) {
            char logMsg[64];
            if (getStrUsb(false, logMsg, sizeof(logMsg))) {
                ESP_LOGI(TAG, "LOG USB: %s", logMsg);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_RATE_MS));
    }
}
