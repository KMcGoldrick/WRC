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
static int logLevel = ESP_LOG_NONE;

// Plot selection
static int serial_plot = 1;

#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include <stdio.h>
#include "driver/gpio.h"

#define RS485_DE_GPIO 5

#define UART_NUM_RS485 2
#define TX_PIN_RS485 8
#define RX_PIN_RS485 9

void redirect_printf_to_uart()
{
	printf("Redirecting printf to UART2...\n");
    // Configure UART2
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_RS485, &uart_config);
    uart_set_pin(UART_NUM_RS485, TX_PIN_RS485, RX_PIN_RS485, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_RS485, 1024, 0, 0, NULL, 0);

    // Redirect printf to UART2
    esp_vfs_dev_uart_use_driver(UART_NUM_RS485);

    // DE pin for RS-485
    gpio_set_direction(RS485_DE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_DE_GPIO, 0); // start in receive mode

	printf("Printf is now redirected to UART2 %d %d.\n", TX_PIN_RS485, RX_PIN_RS485);
}

void init_uart_rs485() {
    // 1. UART config
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_RS485, &uart_config);

    // 2. Set pins
    uart_set_pin(UART_NUM_RS485, TX_PIN_RS485, RX_PIN_RS485, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 3. Install driver
    uart_driver_install(UART_NUM_RS485, 1024, 0, 0, NULL, 0);

    // 4. RS-485 DE
    gpio_set_direction(RS485_DE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_DE_GPIO, 0); // receive mode
}

void send_text_uart(const char* text) {
    // Enable transmit for RS-485
    gpio_set_level(RS485_DE_GPIO, 1);

    uart_write_bytes(UART_NUM_RS485, text, strlen(text));

    // Wait until transmission completes
    uart_wait_tx_done(UART_NUM_RS485, pdMS_TO_TICKS(100));

    // Return to receive mode
    gpio_set_level(RS485_DE_GPIO, 0);
}

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
    printf(">>> APP MAIN STARTED <<<\n");
	init_uart_rs485();
	//redirect_printf_to_uart2();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("Console Loop %d\n", ++loopCounter);
        char uartMessage[50];
        sprintf(uartMessage, "Uart Loop %d\n", loopCounter);
        send_text_uart(uartMessage);
    }

    //esp_log_level_set("*", logLevel);
    //esp_log_level_set(TAG, logLevel);

    //ESP_LOGI(TAG, "==============================================================");
    //ESP_LOGI(TAG, "  WRC System Startup");
    //ESP_LOGI(TAG, "  Build date: " __DATE__ " " __TIME__);

#if defined(CONFIG_IDF_TARGET_ESP32S2)
    //ESP_LOGI(TAG, "  Target: ESP32-S2");
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    //SP_LOGI(TAG, "  Target: ESP32-S3");
#else
    //ESP_LOGI(TAG, "  Target: Unknown ESP32 variant");
#endif

    //ESP_LOGI(TAG, "==============================================================");
    
    // Wait to allow TCM to power up
    //ESP_LOGI(TAG, "Delaying %d ms for TCM startup...", STARTUP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
	//ESP_LOGI(TAG, "Continuing initialization...");

    // Initialize peripherals
    initLED();
    sequenceLED();

    if (useTCM) {
        if (!initUsb(logLevel)) {
            //ESP_LOGE(TAG, "USB initialization failed");
            while (1) {
                setPixelColor(0, 255, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                setPixelColor(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        //ESP_LOGI(TAG, "USB initialized");


        if (!initTcm(logLevel)) {
            //ESP_LOGE(TAG, "TCM initialization failed");
            while (1) {
                setPixelColor(0, 255, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                setPixelColor(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
		}
        //ESP_LOGI(TAG, "TCM   initialized");

    }

    redirect_printf_to_uart();

    //ESP_LOGI(TAG, "Initialization complete. Entering main loop...");

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    while (1) {
        runLED();

        //ESP_LOGI(TAG, "Loop %d", ++loopCounter);
		printf("Loop %d\n", loopCounter);

        if (useTCM) {
            tcmProcessOk = runTcm(serial_plot);
        }

        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_RATE_MS));
    }
}
