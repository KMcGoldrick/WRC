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

// USB selection
static bool useTCM = false;
char uartMessage[192];

// Log/Plot selection
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
static int logLevel = ESP_LOG_NONE; // Set to NONE to plot over RS485
// Plot selection see myTcm.c tcmPlot()
static int serial_plot = 11;

// ----------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------
static led_strip_handle_t led_strip;
static bool tcmProcessOk = true;
static int loopCounter = 0;


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

static void init_log_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = LOG_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    ESP_ERROR_CHECK(uart_driver_install(LOG_UART_NUM, LOG_UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LOG_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LOG_UART_NUM, LOG_TX_PIN, LOG_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static int uart_log_vprintf(const char* fmt, va_list args) {
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
	// add CR
	strcat(buffer, "\r");
	len += 1;
    if (len > 0) {
        uart_write_bytes(LOG_UART_NUM, buffer, len);
    }
    return len;
}

void redirect_esp_log(void) {
    esp_log_set_vprintf(uart_log_vprintf);
}

// ----------------------------------------------------------------------
// Application Entry Point
// ----------------------------------------------------------------------
void app_main(void) {

    if (logLevel == ESP_LOG_NONE) {
        init_rs485();
    }
    else {

        init_log_uart();
        redirect_esp_log();
    }

    // --- Allow 5 seconds for flashing ---
    ESP_LOGI(TAG, "Startup delay: 5 seconds. You can flash the device now...");
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5000 ms = 5 seconds

    //esp_log_level_set("*", logLevel);
    //esp_log_level_set(TAG, logLevel);

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
            //send_rs485("ERR:USB_INIT");

            for (;;) {
                ESP_LOGE(TAG, "USB initialization failed");
                setPixelColor(0, 0, 0, 255);
                vTaskDelay(pdMS_TO_TICKS(500));
                setPixelColor(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        ESP_LOGI(TAG, "USB initialized");

        if (!initTcm(logLevel)) {
            //send_rs485("ERR:TCM_INIT");

            for (;;) {
                ESP_LOGE(TAG, "TCM initialization failed");
                setPixelColor(0, 255, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                setPixelColor(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
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

		ESP_LOGI(TAG, "-----------------------------------");
        ESP_LOGI(TAG, "Loop %d", ++loopCounter);
        uint8_t response[128];

        int len = read_rs485_bytes(response, sizeof(response), 1000);

        if (len > 0)
        {
            ESP_LOG_BUFFER_HEXDUMP("Hexdump:", response, len, ESP_LOG_INFO);
        }

        if (len > 0)
        {
            ESP_LOGI(TAG, "RX: %d bytes", len);

            for (int i = 0; i < len; i++)
                printf("%02X ", response[i]);

            printf("\n");
        }

        if (useTCM) {
            tcmProcessOk = runTcm(serial_plot);
        }
        else 
        {
            snprintf(uartMessage, sizeof(uartMessage),
                "main loop: %d\n", loopCounter);
            send_rs485_text(uartMessage);
        }

        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_RATE_MS));
    }
}
