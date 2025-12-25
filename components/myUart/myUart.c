// ------------------------------
// Standard C Library
// ------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ------------------------------
// FreeRTOS Headers
// ------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// ------------------------------
// ESP-IDF UART Headers
// ------------------------------
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

// ------------------------------
// Project Headers
// ------------------------------
#include "myUart.h"
#include "myTcm.h"
#include "WRCDefs.h"

// ------------------------------
// Constants
// ------------------------------
#define TAG "UART"
#define UART_NUM        UART_NUM_1
#define UART_RX_PIN     19   // Remap to 4
#define UART_TX_PIN     20   // Remap to 5
#define UART_BAUD       115200
#define UART_RX_BUF_LEN 1024
#define UART_TX_BUF_LEN 512
#define UART_RAW_BUF_SIZE 512

// ------------------------------
// Static variables
// ------------------------------
static uint8_t uart_rx_buf[UART_RAW_BUF_SIZE];
static size_t uart_rx_len = 0;
static volatile bool uart_data_ready = false;

static bool uart_initialized = false;

// ------------------------------
// Millis helper
// ------------------------------
unsigned long millisUart() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

// ------------------------------
// Hex to byte helper
// ------------------------------
static uint8_t hex_to_byteUart(const char* hex) {
    char buf[3] = { hex[0], hex[1], 0 };
    return (uint8_t)strtol(buf, NULL, 16);
}

// ------------------------------
// Decode GSR ASCII string
// ------------------------------
int decode_gsr_valuesUart(const char* response,
    uint16_t* temperature,
    int16_t* ax, int16_t* ay, int16_t* az,
    int16_t* mx, int16_t* my, int16_t* mz,
    uint16_t* battery)
{
    if (!response) return -1;

    const char* p = strstr(response, "GSR");
    if (p) {
        p += 3;
        while (*p == ' ' || *p == ':') p++;
    }
    else {
        p = response;
    }

    if (strlen(p) < 40) return -2;

    uint16_t raw[10];
    for (int i = 0; i < 10; i++) {
        uint8_t lo = hex_to_byteUart(p + i * 4);
        uint8_t hi = hex_to_byteUart(p + i * 4 + 2);
        raw[i] = (uint16_t)((hi << 8) | lo);
    }

    if (temperature) *temperature = raw[0];
    if (ax) *ax = (int16_t)raw[1];
    if (ay) *ay = (int16_t)raw[2];
    if (az) *az = (int16_t)raw[3];
    if (mx) *mx = (int16_t)raw[4];
    if (my) *my = (int16_t)raw[5];
    if (mz) *mz = (int16_t)raw[6];
    if (battery) *battery = raw[7];

    return 0;
}

// ------------------------------
// ASCII85 to float
// ------------------------------
float ascii85_to_floatUart(const char str[5]) {
    uint32_t value = 0;
    for (int i = 0; i < 5; i++) {
        if (str[i] < '!' || str[i] > 'u') return NAN;
        value = value * 85 + (str[i] - '!');
    }
    union { uint32_t u; float f; } u;
    u.u = value;
    return u.f;
}

// ------------------------------
// UART RX task
// ------------------------------
static void uart_rx_task(void* arg)
{
    uint8_t buf[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (len > 0) {
            if (len > UART_RAW_BUF_SIZE) len = UART_RAW_BUF_SIZE;
            memcpy(uart_rx_buf, buf, len);
            uart_rx_len = len;
            uart_data_ready = true;

            ESP_LOGV(TAG, "UART RX %d bytes", len);
            for (int i = 0; i < len; i++) {
                ESP_LOGV(TAG, "  Byte %d: 0x%02X", i, buf[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ------------------------------
// Send command over UART
// ------------------------------
esp_err_t send_commandUart(const char* cmd) {
    if (!cmd) return ESP_ERR_INVALID_ARG;

    char cmd_with_cr[64];
    snprintf(cmd_with_cr, sizeof(cmd_with_cr), "%s\r", cmd);

    int len = strlen(cmd_with_cr);
    int written = uart_write_bytes(UART_NUM, cmd_with_cr, len);
    if (written != len) {
        ESP_LOGE(TAG, "UART write failed: wrote %d/%d bytes", written, len);
        return ESP_FAIL;
    }

    ESP_LOGV(TAG, "UART command sent: %s", cmd);
    return ESP_OK;
}

// ------------------------------
// Initialize UART
// ------------------------------
bool initUart(int log_level) {
    esp_log_level_set(TAG, log_level);

    if (uart_initialized) return true;

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_LEN, UART_TX_BUF_LEN, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL, 1);

    uart_initialized = true;
    ESP_LOGI(TAG, "UART initialized (TX=%d RX=%d)", UART_TX_PIN, UART_RX_PIN);
    return true;
}

// ------------------------------
// Enumerate / connect
// ------------------------------
bool enumerate_deviceUart(void) {
    return uart_initialized;
}

bool connectDeviceUart(int vid, int pid) {
    if (!enumerate_deviceUart()) return false;
    ESP_LOGI(TAG, "UART device ready (TX=%d RX=%d)", UART_TX_PIN, UART_RX_PIN);
    return true;
}

// ------------------------------
// Get string response
// ------------------------------
bool getStrUart(char* save_as, size_t save_size, const char* command) {
    if (!save_as || !command || save_size == 0) return false;

    for (int attempt = 0; attempt < NUM_UART_RETRIES; ++attempt) {
        memset(save_as, 0, save_size);
        uart_data_ready = false;

        send_commandUart(command);

        uint32_t start = millisUart();
        while (!uart_data_ready && (millisUart() - start < UART_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!uart_data_ready) continue;

        size_t len = uart_rx_len;
        if (len >= save_size) len = save_size - 1;
        memcpy(save_as, uart_rx_buf, len);
        save_as[len] = '\0';

        uart_data_ready = false;
        uart_rx_len = 0;
        return true;
    }
    return false;
}

// ------------------------------
// Get sensor readings
// ------------------------------
bool getSensorsRawUart(rawSensors* out_sensors, const char* command) {
    if (!out_sensors || !command) return false;

    for (int attempt = 0; attempt < NUM_UART_RETRIES; ++attempt) {
        uart_data_ready = false;
        uart_rx_len = 0;

        send_commandUart(command);

        uint32_t start = millisUart();
        while (!uart_data_ready && (millisUart() - start < UART_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!uart_data_ready) continue;

        const char* str_start = strstr((char*)uart_rx_buf, command);
        if (!str_start) continue;
        str_start += strlen(command) + 1;

        uint16_t temperature = 0, battery = 0;
        int16_t ax = 0, ay = 0, az = 0, mx = 0, my = 0, mz = 0;

        if (decode_gsr_valuesUart(str_start, &temperature, &ax, &ay, &az, &mx, &my, &mz, &battery) != 0)
            continue;

        out_sensors->temp = temperature;
        out_sensors->acc.x = ax; out_sensors->acc.y = ay; out_sensors->acc.z = az;
        out_sensors->mag.x = mx; out_sensors->mag.y = my; out_sensors->mag.z = mz;
        out_sensors->batt = battery;

        uart_data_ready = false;
        uart_rx_len = 0;
        return true;
    }
    return false;
}

// ------------------------------
// Get float via ASCII85
// ------------------------------
bool getFloatAscii85Uart(float* out_value, const char* item, const char* command, const char* address) {
    if (!out_value || !command || !address) return false;

    char full_command[64];
    snprintf(full_command, sizeof(full_command), "%s %s", command, address);

    for (int attempt = 0; attempt < NUM_UART_RETRIES; ++attempt) {
        uart_data_ready = false;
        uart_rx_len = 0;

        send_commandUart(full_command);

        uint32_t start = millisUart();
        while (!uart_data_ready && (millisUart() - start < UART_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!uart_data_ready) continue;

        const char* str_start = strstr((char*)uart_rx_buf, command);
        if (!str_start) continue;

        const char* val_start = strchr(str_start, ' ');
        if (!val_start) continue;
        val_start += 6;

        const char* val_end = strchr(val_start, '\r');
        if (!val_end) val_end = (char*)uart_rx_buf + uart_rx_len;

        size_t len_val = val_end - val_start;
        if (len_val == 0 || len_val >= 32) continue;

        char temp[32];
        memcpy(temp, val_start, len_val);
        temp[len_val] = '\0';

        *out_value = ascii85_to_floatUart(temp);

        uart_data_ready = false;
        uart_rx_len = 0;
        return true;
    }

    return false;
}
