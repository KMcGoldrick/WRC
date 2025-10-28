/*
Open Items
Retries on initialization failures TCM USB
Robust initialize, retry if required, only retry what is necessary
Error handling in run mode
Confirm raw data is received correctly............
Send RS485 to another serial port.
Send serial data only after averaged
MRF TMX TMY TMZ Temperature compensated Mag calibration
READme file
USE LEDs for information
Timing review and optimization
RS485
ESP32S2
Packaging
Cabling
*/


// ------------------------------
// ESP-IDF System Headers
// ------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

// ------------------------------
// Driver / Component Headers
// ------------------------------
#include "led_strip.h"

// ------------------------------
// Project Headers
// ------------------------------
#include "WRCDefs.h"
#include "myNvs.h"
#include "myTcm.h"
#include "myUsb.h"

#define TAG "WRC"

int loopCounter = 0;
led_strip_handle_t led_strip;
bool processRunning = false; // Update based on TCM status
bool erase_nvs_log = false; // Set to true to erase NVS on startup
bool print_nvs_log = false;  // Set to true to print NVS log on startup
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
esp_log_level_t overall_log_level = ESP_LOG_INFO; //Set to control overall log level
bool plotting_all_loops = true; // Set to true to enable serial plotting for all loops
int serial_plot = 3;    // 0 none
                        // 1 Heading and Current 
                        // 2 Roll Pitch Yaw
                        // 3 Accel X Y Z
                        // 4 Mag X Y Z
                        // 5 Temperature Voltage  
						// Values are space separated for easy serial plotting
						// Set to 0 to disable
						// First value is serial_plot value for plotting software
						// Subsequent values depend on mode
						// See myTcm.c for details 

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
    int ledTime_ms = LEDTime_ms;

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

    esp_log_level_set("*", overall_log_level);
	esp_log_level_set(TAG, overall_log_level);

	ESP_LOGI(TAG, "..............................................................");
#ifdef CONFIG_IDF_TARGET_ESP32S2
    ESP_LOGI(TAG, "................Hello from the ESP32-S2!......................");
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
	ESP_LOGI(TAG, "................Hello from the ESP32-S3!......................");
#endif
    ESP_LOGI(TAG, "Build date and time: " __DATE__ " " __TIME__);
    ESP_LOGI(TAG, "..............................................................");

	initLED();
	sequenceLED();
	initNvsLog(erase_nvs_log,print_nvs_log);
    appendNvsLog("\n......Start......\n");
    if (!initUsb()){
        ESP_LOGE(TAG, "USB Initialization Failed");
		appendNvsLog("USB Initialization Failed\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
	}
    if (!initTcm()) {
        ESP_LOGE(TAG, "TCM Initialization Failed");
		appendNvsLog("TCM Initialization Failed\n");
		while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
	}

    while (1) {
        runLED();
        runTcm();

        ESP_LOGV(TAG, "Looping... %d", ++loopCounter);
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_RATE_MS));
    }
}
