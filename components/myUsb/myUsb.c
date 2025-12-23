// myUsb.c
// -----------------------------------------------------------
// -----------------------------------------------------------
// USB-based TCM interface
// -----------------------------------------------------------

// Standard C Library
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// ESP-IDF
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_types_ch9.h"

// Project headers
#include "myUsb.h"
#include "WRCDefs.h"
#include "myTcm.h"

#define TAG "USB"
// ------------------------------
// Buffers & State
// ------------------------------
static char rxBuff[128];
static bool data_available = false;

static cdc_acm_dev_hdl_t tcm_cdc_hdl = NULL;

static usb_host_client_handle_t client_hdl = NULL;
static bool usb_host_initialized = false;

// ------------------------------
// Helpers
// ------------------------------
unsigned long millis(void) {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

static uint8_t hex_to_byte(const char* hex)
{
    char buf[3] = { hex[0], hex[1], 0 };
    return (uint8_t)strtol(buf, NULL, 16);
}

// Decode ASCII85 string to float
float ascii85_to_float(const char str[5]) {
    uint32_t value = 0;
    for (int i = 0; i < 5; i++) {
        if (str[i] < '!' || str[i] > 'u') return NAN;
        value = value * 85 + (str[i] - '!');
    }
    union { uint32_t u; float f; } u;
    u.u = value;
    return u.f;
}

// Decode GSR ASCII string into individual variables
int decode_gsr_values(const char* response,
    uint16_t* temperature,     // Unsigned ADC
    int16_t* ax, int16_t* ay, int16_t* az,  // Signed accel
    int16_t* mx, int16_t* my, int16_t* mz,  // Signed mag
    uint16_t* battery)          // Unsigned ADC
{
    if (!response) return -1;

    // Skip optional "GSR" and whitespace/colon
    const char* p = strstr(response, "GSR");
    if (p) {
        p += 3;
        while (*p == ' ' || *p == ':') p++;
    }
    else {
        p = response;
    }

    // There should be at least 40 ASCII hex chars (20 bytes)
    size_t len = strlen(p);
    if (len < 40) return -2;

    // Decode 10 channels (each 4 hex chars = 2 bytes)
    uint16_t raw[10];
    for (int i = 0; i < 10; i++) {
        uint8_t lo = hex_to_byte(p + i * 4);
        uint8_t hi = hex_to_byte(p + i * 4 + 2);
        raw[i] = (uint16_t)((hi << 8) | lo);
    }

    // Assign to correct types
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
// CDC-ACM callbacks
// ------------------------------
static bool handle_rx(const uint8_t* data, size_t data_len, void* user_arg) {
    if (data_len >= 6 && memcmp(data, "ERR 00", 6) == 0) return false;

    // Copy to buffer (cast away const is safe here)
    strncpy(rxBuff, (const char*)data, sizeof(rxBuff) - 1);
    rxBuff[sizeof(rxBuff) - 1] = '\0';

    data_available = true;
    return true;  // indicate we handled data
}

// ------------------------------
// CDC-ACM device configuration
// ------------------------------
static const cdc_acm_host_device_config_t dev_config = {
    .connection_timeout_ms = 2000,
    .out_buffer_size = 512,
    .user_arg = NULL,
    .event_cb = NULL,
    .data_cb = handle_rx
};

// ------------------------------
// USB Tasks
// ------------------------------
static void usb_event_handler_task(void* arg) {
    while (1) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &event_flags);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "usb_host_lib_handle_events failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void usb_client_task(void* arg) {
    vTaskDelay(pdMS_TO_TICKS(200)); // let host initialize

    usb_host_client_config_t client_config = {
        .is_synchronous = true,
        .max_num_event_msg = 5
    };
    esp_err_t err = usb_host_client_register(&client_config, &client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_client_register failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB Host client registered");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    ESP_LOGI(TAG, "CDC-ACM driver installed");

    usb_host_initialized = true;

    while (1) vTaskDelay(pdMS_TO_TICKS(2000));
}

// ------------------------------
// USB Initialization
// ------------------------------
bool initUsb(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
#ifdef CONFIG_IDF_TARGET_ESP32S3
        .peripheral_map = BIT0
#endif
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host installed");

    xTaskCreatePinnedToCore(usb_event_handler_task, "usb_event_handler", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(usb_client_task, "usb_client_task", 8192, NULL, 5, NULL, 1);

    uint32_t loopCnt = 0;
    while (!usb_host_initialized) {
        if (++loopCnt * USB_HOST_INIT_DELAY_MS >= USB_HOST_TIMEOUT_MS) {
            ESP_LOGE(TAG, "USB host failed to initialize");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(USB_HOST_INIT_DELAY_MS));
    }

    ESP_LOGI(TAG, "USB host initialized");
    return true;
}

// ------------------------------
// Device enumeration and CDC connect
// ------------------------------
static bool enumerate_and_connect(int vid, int pid) {
    uint8_t addr_list[8];
    int addr_count = 0;

    if (usb_host_device_addr_list_fill(8, addr_list, &addr_count) != ESP_OK) return false;
    if (addr_count == 0) return false;

    for (int i = 0; i < addr_count; i++) {
        usb_device_handle_t dev_hdl;
        const usb_device_desc_t* dev_desc;

        if (usb_host_device_open(client_hdl, addr_list[i], &dev_hdl) != ESP_OK) continue;
        if (usb_host_get_device_descriptor(dev_hdl, &dev_desc) != ESP_OK) continue;

        if (dev_desc->idVendor == vid && dev_desc->idProduct == pid) {
            cdc_acm_dev_hdl_t* handle = &tcm_cdc_hdl;
            if (cdc_acm_host_open(dev_desc->idVendor, dev_desc->idProduct, 0, &dev_config, handle) == ESP_OK) {
                ESP_LOGI(TAG, "TCM device connected");
                return true;
            }
        }

        usb_host_device_close(client_hdl, dev_hdl);
    }

    return false;
}

bool connectDevice(int vid, int pid) {
    return enumerate_and_connect(vid, pid);
}

// ------------------------------
// USB Send Command
// ------------------------------
esp_err_t send_command(const char* cmd) {
    data_available = false;
    cdc_acm_dev_hdl_t handle = tcm_cdc_hdl;
    if (!handle) return ESP_FAIL;

    char buf[64];
    snprintf(buf, sizeof(buf), "%s\r", cmd);
    return cdc_acm_host_data_tx_blocking(handle, (const uint8_t*)buf, strlen(buf), 1000);
}

// ------------------------------
// Get string response from USB
// ------------------------------
bool getStrUsb(char* save_as, size_t save_size, const char* command) {
    if (!save_as || !command || save_size == 0) return false;

    for (int attempt = 0; attempt < NUM_USB_RETRIES; ++attempt) {
        memset(save_as, 0, save_size);
        data_available = false;
        memset(rxBuff, 0, sizeof(rxBuff));

        if (send_command(command) != ESP_OK) continue;

        uint32_t start = millis();
        while (!data_available && (millis() - start < USB_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!data_available) continue;

        const char* str_start = strstr(rxBuff, command);
        if (!str_start) continue;

        const char* val_start = strchr(str_start, ' ');
        if (!val_start) continue;
        val_start += 3;

        const char* val_end = strchr(val_start, '\r');
        if (!val_end) continue;

        size_t len = val_end - val_start;
        if (len >= save_size) len = save_size - 1;

        memcpy(save_as, val_start, len);
        save_as[len] = '\0';

        return true;
    }

    return false;
}

// ------------------------------
// Get ASCII85 float from USB
// ------------------------------
bool getFloatAscii85Usb(float* out_value, const char* item, const char* command, const char* address) {
    if (!out_value || !command || !address) return false;

    char full_command[64];
    snprintf(full_command, sizeof(full_command), "%s %s", command, address);

    for (int attempt = 0; attempt < NUM_USB_RETRIES; ++attempt) {
        data_available = false;
        memset(rxBuff, 0, sizeof(rxBuff));

        if (send_command(full_command) != ESP_OK) continue;

        uint32_t start = millis();
        while (!data_available && (millis() - start < USB_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!data_available) continue;

        const char* str_start = strstr(rxBuff, command);
        if (!str_start) continue;

        const char* val_start = strchr(str_start, ' ');
        if (!val_start) continue;
        val_start += 6;

        const char* val_end = strchr(val_start, '\r');
        if (!val_end) continue;

        size_t len = val_end - val_start;
        if (len == 0 || len >= 32) continue;

        char temp[32];
        memcpy(temp, val_start, len);
        temp[len] = '\0';

        *out_value = ascii85_to_float(temp);
        return true;
    }

    return false;
}

bool getSensorsRawUSB(rawSensors* out_sensors, const char* command)
{
    if (!out_sensors || !command) {
        ESP_LOGE(TAG, "getSensorsRawUSB: invalid arguments");
        return false;
    }
    // Send command
    send_command(command);

    // Wait for response (timeout after 3 seconds)
    uint32_t start = millis();
    while (!data_available && (millis() - start < USB_RESPONSE_DELAY_MS)) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!data_available) {
        ESP_LOGE(TAG, "getSensorsRawUSB: timeout waiting for data after %u ms", USB_RESPONSE_DELAY_MS);
        return false;
    }

    // Ensure rxBuff contains something
    if (rxBuff[0] == '\0') {
        ESP_LOGE(TAG, "getSensorsRawUSB: empty rxBuff");
        return false;
    }
    const char* str_start = strstr((const char*)rxBuff, command);
    if (!str_start) {
        ESP_LOGE(TAG, "getSensorsRawUSB: command '%s' not found in rxBuff", command);
        return false;
    }
    str_start += 6; // Skip "GSR 28" prefix
    const char* str_end = strchr(str_start, '\r');
    if (!str_end) {
        ESP_LOGE(TAG, "getSensorsRawUSB: missing CR terminator");
        return false;
    }
    uint16_t temperature, battery;
    int16_t ax, ay, az, mx, my, mz;
    if (decode_gsr_values(str_start,
        &temperature, &ax, &ay, &az,
        &mx, &my, &mz,
        &battery) == 0) {
        out_sensors->temp = temperature;
        out_sensors->acc.x = ax;
        out_sensors->acc.y = ay;
        out_sensors->acc.z = az;
        out_sensors->mag.x = mx;
        out_sensors->mag.y = my;
        out_sensors->mag.z = mz;
        out_sensors->batt = battery;
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Parsed raw sensor readings:");
        ESP_LOGI(TAG, "  Temperature: %d", temperature);
        ESP_LOGI(TAG, "  Acceleration: X=%d Y=%d Z=%d", ax, ay, az);
        ESP_LOGI(TAG, "  Magnetometer: X=%d Y=%d Z=%d", mx, my, mz);
        ESP_LOGI(TAG, "  Battery: %d mV", battery);
    }
    else {
        ESP_LOGE(TAG, "getSensorsRawUSB: Failed to decode sensor readings");
        return false;
    }
    return true;
}