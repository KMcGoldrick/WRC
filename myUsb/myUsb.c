// ------------------------------
// Standard C Library
// ------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ------------------------------
// FreeRTOS Headers
// ------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

char buff[128];
char readings[64] = { 0 };

bool tcm_connected = false;
bool usb_host_initialized = false;

bool get_version = true;
bool get_serialNum = true;
bool get_sensor_readings = true;

// Handle to TCM device
usb_device_handle_t TCMdev_hdl = NULL;
// Configuration descriptor of TCM device
usb_config_desc_t* TCMconfig_desc = NULL;
// Descriptor of TCM device
const usb_device_desc_t* TCMdev_desc = NULL;

// Handle to CDC-ACM device driver
cdc_acm_dev_hdl_t cdc_hdl = NULL;
// Connection handle to USB Host library
usb_host_client_handle_t client_hdl = NULL;

void readSensors(void)
{
    get_sensor_readings = true;
}

void parse_response(const uint8_t* data, size_t data_len) {
    // Firmware version
    const char* fw_start = strstr((const char*)data, FIRMWARE_VERSION_CMD);
    if (fw_start) {
        fw_start = strchr(fw_start, ' ');
        if (fw_start) {
            fw_start += 3; // Point to version string
            const char* fw_end = strchr(fw_start, '\r');
            if (fw_end) {
                size_t len = fw_end - fw_start;
                if (len < sizeof(tcmInfo.version)) {
                    strncpy(tcmInfo.version, fw_start, len);
                    tcmInfo.version[len] = '\0';
                }
            }
        }
		get_version = false;
		ESP_LOGI(TAG, "Parsed Firmware version: %s", tcmInfo.version);
    }

    // Serial number
    const char* sn_start = strstr((const char*)data, SERIAL_NUMBER_CMD);
    if (sn_start) {
        sn_start = strchr(sn_start, ' ');
        if (sn_start) {
            sn_start += 3; // Point to serial number string
            const char* sn_end = strchr(sn_start, '\r');
            if (sn_end) {
                size_t len = sn_end - sn_start;
                if (len < sizeof(tcmInfo.serialNum)) {
                    strncpy(tcmInfo.serialNum, sn_start, len);
                    tcmInfo.serialNum[len] = '\0';
                }
            }
        }
		get_serialNum = false;
        ESP_LOGI(TAG, "Parsed Serial Number: %s", tcmInfo.serialNum);
    }

    // Sensor readings
    const char* sr_start = strstr((const char*)data, SENSOR_READINGS_CMD);
    if (sr_start) {
            sr_start += 6; // Skip "GSR 28" prefix
            const char* sr_end = strchr(sr_start, '\r');
            // sr_start points to the ASCII hex string, e.g. "3986b8ee600259ff..."
            if (!sr_end) {
                ESP_LOGE(TAG, "Malformed sensor readings response");
                return;
            }
            else {
                ESP_LOGI(TAG, "String:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, sr_start, sr_end - sr_start, ESP_LOG_INFO);
            }


            get_sensor_readings = false;
    }
}

void handle_rx(uint8_t* data, size_t data_len, void* user_arg)
{
    // Only log if the received data is not "ERR 00"
    if (data_len == 8 && memcmp(data, "ERR 00..", 6) == 0) {
        // Ignore "ERR 00.." responses
        return;
    }
    ESP_LOGI(TAG, "Data received");
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
    // If you need to process data, do it here.
	parse_response(data, data_len);
}

// CDC-ACM configuration
const cdc_acm_host_device_config_t dev_config = {
    .connection_timeout_ms = 2000,
    .out_buffer_size = 512,
    .user_arg = NULL,
    .event_cb = NULL,
    .data_cb = handle_rx
};

bool enumerate_TCM_device(void)
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
        ESP_LOGI(TAG, "No USB devices found");
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
            if (dev_desc->idVendor == TCM_VID && dev_desc->idProduct == TCM_PID) {
                ESP_LOGI(TAG, "TCM device found at address %d", addr);
                TCMdev_hdl = dev_hdl;
                TCMdev_desc = dev_desc;
            }
            else
            {
                ESP_LOGI(TAG, "\n[DEVICE %d]", addr);
                ESP_LOGI(TAG, "  VID=0x%04X, PID=0x%04X, bcdUSB=0x%04X",
                    dev_desc->idVendor, dev_desc->idProduct, dev_desc->bcdUSB);
                ESP_LOGI(TAG, "  Class=0x%02X, SubClass=0x%02X, Protocol=0x%02X",
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
            TCMconfig_desc = (usb_config_desc_t*)config_desc; // Save for later use
            ESP_LOGI(TAG, "bNumInterfaces = %d", config_desc->bNumInterfaces);

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
                            ESP_LOGI(TAG,
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
                        ESP_LOGI(TAG,
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

	return (TCMdev_hdl != NULL);
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

esp_err_t send_command(const char* cmd)
{
    if (cdc_hdl == NULL) {
        ESP_LOGE(TAG, "CDC handle not initialized");
        return ESP_FAIL;
    }

    char cmd_with_cr[32];
    snprintf(cmd_with_cr, sizeof(cmd_with_cr), "%s\r", cmd);
    size_t len = strlen(cmd_with_cr);
    esp_err_t err = cdc_acm_host_data_tx_blocking(cdc_hdl, (const uint8_t*)cmd_with_cr, len, 1000);    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(err));
        return err;
    }
	ESP_LOGI(TAG, "Sent command: %s", cmd);

    return err;
}

bool connect_and_switch_TCM(usb_host_client_handle_t client_hdl,
    const usb_device_desc_t* dev_desc,
    const usb_config_desc_t* config_desc)
{
	bool success = false;
    esp_err_t err;

    if (client_hdl == NULL) {
        ESP_LOGE("USB", "Client handle is NULL");
        return false;
	}
    if (dev_desc == NULL) {
        ESP_LOGE("USB", "Device descriptor is NULL");
        return false;
	}
    if (config_desc == NULL) {
        ESP_LOGE("USB", "Configuration descriptor is NULL");
        return false;
    }
    if (config_desc->bNumInterfaces == 0) {
        ESP_LOGE("USB", "No interfaces found in configuration descriptor");
        return false;
	}

    const uint8_t* p = config_desc->val;
    const uint8_t* end = p + config_desc->wTotalLength;

    while (p < end) {
        const usb_standard_desc_t* desc = (const usb_standard_desc_t*)p;

        if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t* intf = (const usb_intf_desc_t*)p;
            ESP_LOGI("USB", "Interface %d: Class=0x%02X SubClass=0x%02X Protocol=0x%02X NumEP=%d",
                intf->bInterfaceNumber, intf->bInterfaceClass,
                intf->bInterfaceSubClass, intf->bInterfaceProtocol,
                intf->bNumEndpoints);

            if (intf->bInterfaceClass == 0x02 || intf->bInterfaceClass == 0x0A) {
                ESP_LOGI("USB", "Attempting to open CDC interface %d", intf->bInterfaceNumber);

                err = cdc_acm_host_open(dev_desc->idVendor,
                    dev_desc->idProduct,
                    intf->bInterfaceNumber,
                    &dev_config,
                    &cdc_hdl);

                if (err == ESP_OK) {
                    ESP_LOGI("USB", "Opened CDC interface successfully");
                    success = true;
                }
                else {
                    ESP_LOGE("USB", "Failed to open CDC interface: %s", esp_err_to_name(err));
                }
                break;
            }
        }

        p += desc->bLength;
    }
    if (!success) {
        ESP_LOGE("USB", "No CDC interface found on TCM device");
        return false;
    }
    else {
		return true;
    }
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
        if (!tcm_connected) {
            if (enumerate_TCM_device()) {
                if (connect_and_switch_TCM(client_hdl, TCMdev_desc, TCMconfig_desc)) {
                    send_command(STOP_CMD);
                    tcm_connected = true;
                }
            }
            else {
                ESP_LOGI(TAG, "TCM device not found");
            }
        }
        else {
			if (get_version){
				send_command(FIRMWARE_VERSION_CMD);
			}
			else if (get_serialNum) {
				send_command(SERIAL_NUMBER_CMD);
			}
            else if (get_sensor_readings) {
                send_command(SENSOR_READINGS_CMD);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void initUsb(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
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
		ESP_LOGI(TAG, "Waiting for USB host to initialize...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
	ESP_LOGI(TAG, "USB host initialized");

    while (!tcm_connected) {
        ESP_LOGI(TAG, "Waiting for TCM to connect...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "TCM connected");
}
