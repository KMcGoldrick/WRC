/*
* 
* Open Items
* -----------
* - WS2812 vs SK6812 LED model selection
* - Send RS485 to another serial port
* - Send serial data only after averaging
* - Timing review and optimization
* - RS485, Packaging, Cabling
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
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
static bool tcmProcessOk = true; // Updated by TCM status
static bool erase_nvs_log = false; // Set to true to erase NVS log on startup
static bool print_nvs_log = false; // Set to true to print NVS log on startup
static int loopCounter = 0;

// Available log levels: ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO, ESP_LOG_DEBUG, ESP_LOG_VERBOSE
esp_log_level_t overall_log_level = ESP_LOG_INFO; // Default log level

// Plotting (RS485) options
bool plotting_all_loops = true; // true: plot every loop, false: plot only averaged data
int serial_plot = 3;  // 0: none, 1: heading/current, 2: roll/pitch/yaw, etc. (see myTcm.c tcmPlot() for details)

// ----------------------------------------------------------------------
// LED Helpers
// ----------------------------------------------------------------------
static void setPixelColor(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(led_strip, idx, r, g, b);
    led_strip_refresh(led_strip);
}

static void initLED(void)
{
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

static void sequenceLED(void)
{
    // Non-blocking startup indicator (~600 ms total)
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

static void runLED(void)
{
    static uint32_t lastToggle = 0;
    static bool ledState = false;

    uint32_t now = millis();

    if (now - lastToggle >= LED_TIME_MS) {
        lastToggle = now;
        ledState = !ledState;

        for (int i = 0; i < NUM_LEDS; i++) {
            if (!tcmProcessOk) {
                // TCM error state
                setPixelColor(i, 100, 0, 0); // red
            }
            else if (ledState)
                setPixelColor(i, 0, 100, 0); // green
            else
				setPixelColor(i, 0, 0, 0); // off
        }
    }
}

// ----------------------------------------------------------------------
// Application Entry Point
// ----------------------------------------------------------------------
void app_main(void)
{
    esp_log_level_set("*", overall_log_level);
    esp_log_level_set(TAG, overall_log_level);

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

    ESP_LOGI(TAG, "==============================================================");

    // Wait to allow TCM to power up
	ESP_LOGI(TAG, "Delaying %d ms for TCM startup...", STARTUP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

    // Initialize peripherals
    initLED();
    sequenceLED();

    initNvsLog(erase_nvs_log, print_nvs_log);
    appendNvsLog("\n......Start......\n");

    if (!initUsb()) {
        ESP_LOGE(TAG, "USB Initialization Failed");
        appendNvsLog("USB Initialization Failed\n");
        // Flash red LED for error state
        while (1) {
            setPixelColor(0, 255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            setPixelColor(0, 0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    if (!initTcm()) {
        ESP_LOGE(TAG, "TCM Initialization Failed");
        appendNvsLog("TCM Initialization Failed\n");
        // Flash yellow LED for error
        while (1) {
            setPixelColor(0, 255, 255, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            setPixelColor(0, 0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    ESP_LOGI(TAG, "Initialization complete. Entering main loop...");

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    while (1) {
        runLED();
        tcmProcessOk = runTcm();

        ESP_LOGV(TAG, "Looping... %d", ++loopCounter);
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_RATE_MS));
    }
}
