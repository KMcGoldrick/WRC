// ------------------------------
// Standard C Library
// ------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

// ------------------------------
// FreeRTOS Headers
// ------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// ------------------------------
// ESP-IDF Headers
// ------------------------------
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_types_ch9.h"

// ------------------------------
// Project Headers
// ------------------------------
#include "myUsb.h"
#include "myTcm.h"
#include "WRCDefs.h"

#define TAG "USB"

char rxBuff[128];
char readings[64] = { 0 };
bool data_available = false;

bool device_connected = false;
bool usb_host_initialized = false;

// Handle to connected device
usb_device_handle_t device_dev_hdl = NULL;
// Configuration descriptor of connected device
usb_config_desc_t* device_config_desc = NULL;
// Descriptor of connected device
const usb_device_desc_t* device_desc = NULL;

// Handle to CDC-ACM device driver
cdc_acm_dev_hdl_t cdc_hdl = NULL;
// Connection handle to USB Host library
usb_host_client_handle_t client_hdl = NULL;

unsigned long millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

static uint8_t hex_to_byte(const char* hex)
{
    char buf[3] = { hex[0], hex[1], 0 };
    return (uint8_t)strtol(buf, NULL, 16);
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

esp_err_t send_command(const char* cmd)
{
    ESP_LOGV(TAG, "Sending command: %s", cmd);
	data_available = false;
    if (cdc_hdl == NULL) {
        ESP_LOGE(TAG, "CDC handle not initialized");
        return ESP_FAIL;
    }

    char cmd_with_cr[32];
    snprintf(cmd_with_cr, sizeof(cmd_with_cr), "%s\r", cmd);
    size_t len = strlen(cmd_with_cr);
    esp_err_t err = cdc_acm_host_data_tx_blocking(cdc_hdl, (const uint8_t*)cmd_with_cr, len, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s %s", cmd, esp_err_to_name(err));
        return err;
    }
    ESP_LOGV(TAG, "Sent command: %s", cmd);

    return err;
}

// Decode 5 ASCII85 characters into a 4-byte float
float ascii85_to_float(const char str[5]) {
    uint32_t value = 0;

    // Convert 5 ASCII85 characters to 32-bit integer
    for (int i = 0; i < 5; i++) {
        if (str[i] < '!' || str[i] > 'u') return NAN; // invalid ASCII85
        value = value * 85 + (str[i] - '!');
    }

    // Now value is the 32-bit representation of the float
    union {
        uint32_t u;
        float f;
    } u;
    u.u = value;
    return u.f;
}

void handle_rx(uint8_t* data, size_t data_len, void* user_arg)
{
    // Only log if the received data is not "ERR 00"
    if (data_len == 8 && memcmp(data, "ERR 00..", 6) == 0) {
        // Ignore "ERR 00.." responses
        return;
    }
    ESP_LOGV(TAG, "Data received");
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_VERBOSE);
    strcpy(rxBuff, (char*)data);
	data_available = true;
}

// CDC-ACM configuration
const cdc_acm_host_device_config_t dev_config = {
    .connection_timeout_ms = 2000,
    .out_buffer_size = 512,
    .user_arg = NULL,
    .event_cb = NULL,
    .data_cb = handle_rx
};

bool enumerate_device(int vid, int pid)
{
    uint8_t addr_list[8];
    int addr_count = 0;
    bool showConfigDescriptor = false;

    esp_err_t err = usb_host_device_addr_list_fill(8, addr_list, &addr_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device address list: %s", esp_err_to_name(err));
        return false;
    }

    if (addr_count == 0) {
        ESP_LOGV(TAG, "No USB devices found");
        return false;
    }

    ESP_LOGI(TAG, "Found %d USB device(s)", addr_count);

    for (int i = 0; i < addr_count; i++)
    {
        uint8_t addr = addr_list[i];
        usb_device_handle_t dev_hdl;
        const usb_device_desc_t* dev_desc;
        const usb_config_desc_t* config_desc;

        err = usb_host_device_open(client_hdl, addr, &dev_hdl);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open device addr %d: %s", addr, esp_err_to_name(err));
            continue;
        }

        // --- DEVICE DESCRIPTOR ---
        err = usb_host_get_device_descriptor(dev_hdl, &dev_desc);
        if (err == ESP_OK) {
            if (dev_desc->idVendor == vid && dev_desc->idProduct == pid) {
                ESP_LOGI(TAG, "VID %d PID %d device found at address %d", vid, pid, addr);
                device_dev_hdl = dev_hdl;
                device_desc = dev_desc;
            }
            else
            {
                ESP_LOGV(TAG, "Looking for VID 0x%04X PID 0x%04X, found VID 0x%04X PID 0x%04X at address %d",
					vid, pid, dev_desc->idVendor, dev_desc->idProduct, addr);
                ESP_LOGV(TAG, "\n[DEVICE %d]", addr);
                ESP_LOGV(TAG, "  VID=0x%04X, PID=0x%04X, bcdUSB=0x%04X",
                    dev_desc->idVendor, dev_desc->idProduct, dev_desc->bcdUSB);
                ESP_LOGV(TAG, "  Class=0x%02X, SubClass=0x%02X, Protocol=0x%02X",
                    dev_desc->bDeviceClass, dev_desc->bDeviceSubClass, dev_desc->bDeviceProtocol);
                usb_host_device_close(client_hdl, dev_hdl);
            }
        }
        else {
            ESP_LOGE(TAG, "  Failed to read device descriptor for addr %d", addr);
        }

        // --- CONFIGURATION DESCRIPTOR ---
        err = usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
        if (err == ESP_OK && config_desc)
        {
            device_config_desc = (usb_config_desc_t*)config_desc; // Save for later use
            ESP_LOGV(TAG, "bNumInterfaces = %d", config_desc->bNumInterfaces);

            int offset = config_desc->bLength; // start after config descriptor
            const usb_standard_desc_t* desc = NULL;

            if (showConfigDescriptor)
            {
                while ((desc = usb_parse_next_descriptor(
                    (const usb_standard_desc_t*)config_desc,
                    config_desc->wTotalLength,
                    &offset)) != NULL)
                {
                    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
                        const usb_intf_desc_t* intf = (const usb_intf_desc_t*)desc;

                        // Only print first alternate setting for each logical interface
                        if (intf->bAlternateSetting == 0) {
                            ESP_LOGV(TAG,
                                "  [Interface %d] Class=0x%02X, SubClass=0x%02X, Protocol=0x%02X, NumEP=%d",
                                intf->bInterfaceNumber,
                                intf->bInterfaceClass,
                                intf->bInterfaceSubClass,
                                intf->bInterfaceProtocol,
                                intf->bNumEndpoints);
                        }
                    }
                    else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT)
                    {
                        const usb_ep_desc_t* ep = (const usb_ep_desc_t*)desc;
                        // Only log endpoints if the previous interface's bAlternateSetting == 0
                        // (assumes interfaces and their endpoints appear sequentially)
                        ESP_LOGV(TAG,
                            "    Endpoint Addr=0x%02X, Attr=0x%02X, MaxPkt=%d, Interval=%d",
                            ep->bEndpointAddress,
                            ep->bmAttributes,
                            ep->wMaxPacketSize,
                            ep->bInterval);
                    }

                }
            }
        }

    }

	return (device_dev_hdl != NULL);
}

void usb_event_handler_task(void* arg)
{
    while (1) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &event_flags);

        if (err == ESP_OK) {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
                ESP_LOGW(TAG, "No clients using USB Host");
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
                ESP_LOGW(TAG, "All devices freed");
        }
        else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "usb_host_lib_handle_events failed: %s", esp_err_to_name(err));
        }
    }
}

bool connect_and_switch_device(usb_host_client_handle_t client_hdl,
    const usb_device_desc_t* dev_desc,
    const usb_config_desc_t* config_desc)
{
	bool success = false;
    esp_err_t err;

    if (client_hdl == NULL) {
        ESP_LOGE(TAG, "Client handle is NULL");
        return false;
	}
    if (dev_desc == NULL) {
        ESP_LOGE(TAG, "Device descriptor is NULL");
        return false;
	}
    if (config_desc == NULL) {
        ESP_LOGE(TAG, "Configuration descriptor is NULL");
        return false;
    }
    if (config_desc->bNumInterfaces == 0) {
        ESP_LOGE(TAG, "No interfaces found in configuration descriptor");
        return false;
	}

    const uint8_t* p = config_desc->val;
    const uint8_t* end = p + config_desc->wTotalLength;

    while (p < end) {
        const usb_standard_desc_t* desc = (const usb_standard_desc_t*)p;

        if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t* intf = (const usb_intf_desc_t*)p;
            ESP_LOGV(TAG, "Interface %d: Class=0x%02X SubClass=0x%02X Protocol=0x%02X NumEP=%d",
                intf->bInterfaceNumber, intf->bInterfaceClass,
                intf->bInterfaceSubClass, intf->bInterfaceProtocol,
                intf->bNumEndpoints);

            if (intf->bInterfaceClass == 0x02 || intf->bInterfaceClass == 0x0A) {
                ESP_LOGI(TAG, "Attempting to open CDC interface %d", intf->bInterfaceNumber);

                err = cdc_acm_host_open(dev_desc->idVendor,
                    dev_desc->idProduct,
                    intf->bInterfaceNumber,
                    &dev_config,
                    &cdc_hdl);

                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Opened CDC interface successfully");
                    success = true;
                }
                else {
                    ESP_LOGE(TAG, "Failed to open CDC interface: %s", esp_err_to_name(err));
                }
                break;
            }
        }

        p += desc->bLength;
    }
    if (!success) {
        ESP_LOGE(TAG, "No CDC interface found on connected device");
        return false;
    }
	return success;
}

void usb_client_task(void* arg)
{
    vTaskDelay(pdMS_TO_TICKS(200)); // let the host initialize first

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

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

bool initUsb(void)
{
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
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);//overall_log_level);
    usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .skip_phy_setup = false,
#ifdef CONFIG_IDF_TARGET_ESP32S3
        .peripheral_map = BIT0  // use USB port 0 on S3
#endif
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host installed");

    xTaskCreatePinnedToCore(usb_event_handler_task, "usb_event_handler", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(usb_client_task, "usb_client_task", 8192, NULL, 5, NULL, 1);

    while (!usb_host_initialized) {
		ESP_LOGV(TAG, "Waiting for USB host to initialize...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
	ESP_LOGI(TAG, "USB host initialized");
    return true;

    /*
    while (!tcm_connected) {
        ESP_LOGV(TAG, "Waiting for TCM to connect...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "TCM connected");
    */
}

bool connectDevice(int vid, int pid)
{
	ESP_LOGI(TAG, "Connecting to device VID=0x%04X PID=0x%04X", vid, pid);
    if (enumerate_device(vid, pid)) {
        if (connect_and_switch_device(client_hdl, device_desc, device_config_desc)) {
            ESP_LOGI(TAG, "Device connected and switched to CDC-ACM");
            return true;
        }
	}
    return false;
}

bool getStrUsb(char* save_as, size_t save_size, const char* command)
{
    if (!save_as || !command || save_size == 0) {
        ESP_LOGE(TAG, "getStrUsb: invalid arguments");
        return false;
    }

    for (int attempt = 0; attempt < NUM_RETRIES; ++attempt) {

        // Clear output and temp buffers
        memset(save_as, 0, save_size);
        data_available = false;
        memset(rxBuff, 0, sizeof(rxBuff));

        // Send command
        send_command(command);
        ESP_LOGI(TAG, "getStrUsb: sent command '%s' (try %d/%d)", command, attempt + 1, NUM_RETRIES);

        // Wait for response with timeout
        uint32_t start = millis();
        while (!data_available && (millis() - start < USB_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!data_available) {
            ESP_LOGW(TAG, "getStrUsb: timeout waiting for response (%u ms)", USB_RESPONSE_DELAY_MS);
            continue; // retry
        }

        // Validate received buffer
        if (rxBuff[0] == '\0') {
            ESP_LOGW(TAG, "getStrUsb: empty response buffer");
            continue; // retry
        }

        // Find command in response
        const char* str_start = strstr((const char*)rxBuff, command);
        if (!str_start) {
            ESP_LOGW(TAG, "getStrUsb: command '%s' not found in response", command);
            continue; // retry
        }

        // Find first space after command
        const char* fw_start = strchr(str_start, ' ');
        if (!fw_start) {
            ESP_LOGW(TAG, "getStrUsb: no space found after command");
            continue; // retry
        }

		fw_start += 3; // Move past space and 2-char address

        // Find CR (end of line)
        const char* fw_end = strchr(fw_start, '\r');
        if (!fw_end) {
            ESP_LOGW(TAG, "getStrUsb: missing CR terminator");
            continue; // retry
        }

        // Extract substring safely
        size_t len = fw_end - fw_start;
        if (len >= save_size)
            len = save_size - 1;

        memcpy(save_as, fw_start, len);
        save_as[len] = '\0';

        ESP_LOGI(TAG, "getStrUsb: parsed '%s' = '%s'", command, save_as);
        return true; // success
    }

    ESP_LOGE(TAG, "getStrUsb: failed after %d retries", NUM_RETRIES);
    return false;
}

bool getSensorsRawUSB(rawSensors* out_sensors, const char* command)
{
    if (!out_sensors || !command) {
        ESP_LOGE(TAG, "getSensorsRawUSB: invalid arguments");
        return false;
    }

    for (int attempt = 1; attempt <= NUM_RETRIES; ++attempt) {

        // Reset state
        data_available = false;
        memset(rxBuff, 0, sizeof(rxBuff));

        // Send command
        send_command(command);
        ESP_LOGI(TAG, "getSensorsRawUSB: sent command '%s' (attempt %d/%d)",
            command, attempt, NUM_RETRIES);

        // Wait for response with timeout
        uint32_t start = millis();
        while (!data_available && (millis() - start < USB_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!data_available) {
            ESP_LOGW(TAG, "getSensorsRawUSB: timeout waiting for response (%u ms)",
                USB_RESPONSE_DELAY_MS);
            continue; // retry
        }

        if (rxBuff[0] == '\0') {
            ESP_LOGW(TAG, "getSensorsRawUSB: empty rxBuff");
            continue; // retry
        }

        // Find command text in response
        const char* str_start = strstr((const char*)rxBuff, command);
        if (!str_start) {
            ESP_LOGW(TAG, "getSensorsRawUSB: command '%s' not found in response", command);
            continue; // retry
        }

        // Move past the command and optional space
        str_start += 6;

        // Find end of line
        const char* str_end = strchr(str_start, '\r');
        if (!str_end) {
            ESP_LOGW(TAG, "getSensorsRawUSB: missing CR terminator");
            continue; // retry
        }

        // Temporary decoded values
        uint16_t temperature = 0, battery = 0;
        int16_t ax = 0, ay = 0, az = 0, mx = 0, my = 0, mz = 0;

        if (decode_gsr_values(str_start,
            &temperature, &ax, &ay, &az,
            &mx, &my, &mz,
            &battery) != 0)
        {
            ESP_LOGW(TAG, "getSensorsRawUSB: failed to decode sensor readings");
            continue; // retry
        }

        // Assign parsed values to output
        out_sensors->temp = temperature;
        out_sensors->acc.x = ax;
        out_sensors->acc.y = ay;
        out_sensors->acc.z = az;
        out_sensors->mag.x = mx;
        out_sensors->mag.y = my;
        out_sensors->mag.z = mz;
        out_sensors->batt = battery;

        ESP_LOGI(TAG, "getSensorsRawUSB: success on attempt %d", attempt);
        ESP_LOGI(TAG, "  Temperature: %u", temperature);
        ESP_LOGI(TAG, "  Accel: X=%d  Y=%d  Z=%d", ax, ay, az);
        ESP_LOGI(TAG, "  Mag:   X=%d  Y=%d  Z=%d", mx, my, mz);
        ESP_LOGI(TAG, "  Battery: %u mV", battery);
        return true; // success!
    }

    // All retries failed
    ESP_LOGE(TAG, "getSensorsRawUSB: failed after %d retries", NUM_RETRIES);
    return false;
}

bool getFloatAscii85Usb(float* out_value, const char* item, const char* command, const char* address)
{
    if (!out_value || !command || !address) {
        ESP_LOGE(TAG, "getFloatAscii85Usb: invalid arguments");
        return false;
    }

    char full_command[64];
    snprintf(full_command, sizeof(full_command), "%s %s", command, address);

    for (int attempt = 1; attempt <= NUM_RETRIES; ++attempt) {

        // Reset state before sending
        data_available = false;
        memset(rxBuff, 0, sizeof(rxBuff));

        // Send the command
        send_command(full_command);
        ESP_LOGI(TAG, "getFloatAscii85Usb: sent '%s' (attempt %d/%d)",
            full_command, attempt, NUM_RETRIES);

        // Wait for response
        uint32_t start = millis();
        while (!data_available && (millis() - start < USB_RESPONSE_DELAY_MS)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!data_available) {
            ESP_LOGW(TAG, "getFloatAscii85Usb: timeout waiting for data (%u ms)", USB_RESPONSE_DELAY_MS);
            continue; // retry
        }

        // Validate received buffer
        if (rxBuff[0] == '\0') {
            ESP_LOGW(TAG, "getFloatAscii85Usb: empty rxBuff");
            continue; // retry
        }

        // Ensure command appears in response
        const char* str_start = strstr((const char*)rxBuff, command);
        if (!str_start) {
            ESP_LOGW(TAG, "getFloatAscii85Usb: command '%s' not found in rxBuff", command);
            continue; // retry
        }

        // Optional item validation (skip if wildcard)
        if (item && strcmp(item, "***") != 0) {
            const char* item_start = strstr((const char*)rxBuff, item);
            if (!item_start) {
                ESP_LOGW(TAG, "getFloatAscii85Usb: item '%s' not found in rxBuff", item);
                continue; // retry
            }
        }

        // Find the space after command
        const char* val_start = strchr(str_start, ' ');
        if (!val_start) {
            ESP_LOGW(TAG, "getFloatAscii85Usb: missing space after command");
            continue; // retry
        }

        // Skip spaces and address text
		val_start += 6; // move past ...space and address

        // Find end of value (CR)
        const char* val_end = strchr(val_start, '\r');
        if (!val_end) {
            ESP_LOGW(TAG, "getFloatAscii85Usb: missing CR terminator");
            continue; // retry
        }

        // Extract substring safely
        size_t len = val_end - val_start;
        if (len == 0 || len >= 32) {
            ESP_LOGW(TAG, "getFloatAscii85Usb: invalid ASCII85 length %u", (unsigned)len);
            continue; // retry
        }

        char temp[32];
        memcpy(temp, val_start, len);
        temp[len] = '\0';

        // Convert ASCII85 string to float
        ESP_LOGV(TAG, "getFloatAscii85Usb: converting '%s' to float", temp);
        *out_value = ascii85_to_float(temp);
        ESP_LOGI(TAG, "getFloatAscii85Usb: parsed %s (%s) = %f", item, address, *out_value);
        return true;
    }

    ESP_LOGE(TAG, "getFloatAscii85Usb: failed after %d retries", NUM_RETRIES);
    return false;
}
