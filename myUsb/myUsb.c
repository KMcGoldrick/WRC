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
bool get_calibrations = true;
bool get_sensor_readings = false;

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

bool areSensorsReady(void)
{
    return !get_sensor_readings;
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

// --- Parse float from TCM response buffer ---
bool parse_float_value(const uint8_t* data, float* save_to)
{
    if (!data || !save_to) return false;

	char float_bytes[6] = { 0 }; // 5 bytes + null terminator
    float_bytes[0] = data[11];
    float_bytes[1] = data[12];
    float_bytes[2] = data[14];
    float_bytes[3] = data[14];
    float_bytes[4] = data[15];
    float_bytes[5] = '\0';
	ESP_LOGI(TAG, "Extracted ASCII85 float string %s", float_bytes);

    *save_to = ascii85_to_float(float_bytes);
    ESP_LOGI(TAG, "Parsed float value: %f", *save_to);

    return true;
}

// --- Check for keyword, parse value, send next calibration ---
bool check_send_next(const uint8_t* data, const char* keyword, const char* next_calib_addr, float* save_to)
{
    if (!data || !keyword || !next_calib_addr) {
        ESP_LOGW(TAG, "check_send_next: null argument");
        return false;
    }

    // Check if keyword is present
    const char* p = strstr((const char*)data, keyword);
    if (!p) return false;

    ESP_LOGI(TAG, "Parsed %s", keyword);

    // Parse float if requested
    if (save_to != NULL) {
        if (!parse_float_value(data, save_to)) {
            ESP_LOGW(TAG, "Could not parse value for %s", keyword);
        }
    }

    // Send next calibration command (except for HSE)
    if (strcmp(keyword, "HSE") != 0) {
        char calib_cmd[32];
        snprintf(calib_cmd, sizeof(calib_cmd), "%s %s", CALIBRATION_CMD, next_calib_addr);
        ESP_LOGV(TAG, "Sending command: %s", calib_cmd);
        send_command(calib_cmd);
    }

    return true;
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
                ESP_LOGV(TAG, "String:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, sr_start, sr_end - sr_start, ESP_LOG_VERBOSE);
                uint16_t temperature, battery;
                int16_t ax, ay, az, mx, my, mz;
                if (decode_gsr_values(sr_start,
                    &temperature, &ax, &ay, &az,
                    &mx, &my, &mz,
                    &battery) == 0) {
                    tcmInfo.raw.temp = temperature;
                    tcmInfo.raw.acc.x = ax;
                    tcmInfo.raw.acc.y = ay;
                    tcmInfo.raw.acc.z = az;
                    tcmInfo.raw.mag.x = mx;
                    tcmInfo.raw.mag.y = my;
                    tcmInfo.raw.mag.z = mz;
                    tcmInfo.raw.batt  = battery;
                    ESP_LOGI(TAG, "Parsed Sensor Readings:");
                    ESP_LOGI(TAG, "  Temperature: %d", temperature);
					ESP_LOGI(TAG, "  Temperature: %x", temperature);
                    ESP_LOGI(TAG, "  Acceleration: X=%d Y=%d Z=%d", ax, ay, az);
                    ESP_LOGI(TAG, "  Acceleration: X=%x Y=%x Z=%x", ax, ay, az);
                    ESP_LOGI(TAG, "  Magnetometer: X=%d Y=%d Z=%d", mx, my, mz);
                    ESP_LOGI(TAG, "  Magnetometer: X=%x Y=%x Z=%x", mx, my, mz);
                    ESP_LOGI(TAG, "  Battery: %d mV", battery);
                    ESP_LOGI(TAG, "  Battery: %x mV", battery);
                }
                else {
                    ESP_LOGE(TAG, "Failed to decode sensor readings");
				}
            }
            get_sensor_readings = false;
    }

    // Calibrations
    const char* cal_start = strstr((const char*)data, CALIBRATION_CMD);
    if (cal_start) {
        cal_start += 6; // Skip "RHS 30" prefix
        const char* cal_end = strchr(cal_start, '\r');
        if (!cal_end) {
            ESP_LOGE(TAG, "Malformed calibration response");
            return;
        }
        else {
            ESP_LOGV(TAG, "String:");
            ESP_LOG_BUFFER_HEXDUMP(TAG, cal_start, cal_end - cal_start, ESP_LOG_VERBOSE);
            if (check_send_next(data, "HSSRVN", "06080008", NULL)) { return; }
            if (check_send_next(data, "TMO", "06100008",&tcmInfo.tempCal.TMO)) { return; }
            if (check_send_next(data, "TMR", "06180008",&tcmInfo.tempCal.TMR)) { return; }
			if (check_send_next(data, "TMA", "06200008",&tcmInfo.tempCal.TMA)) { return; }
			if (check_send_next(data, "TMB", "06280008",&tcmInfo.tempCal.TMB)) { return; }
            if (check_send_next(data, "TMC", "06300008",&tcmInfo.tempCal.TMC)) { return; }
			if (check_send_next(data, "AXX", "06380008",&tcmInfo.accCal.AXX)) { return; }
			if (check_send_next(data, "AXY", "06400008",&tcmInfo.accCal.AXY)) { return; }
			if (check_send_next(data, "AXZ", "06480008",&tcmInfo.accCal.AXZ)) { return; }
            if (check_send_next(data, "AYX", "06500008",&tcmInfo.accCal.AYX)) { return; }
            if (check_send_next(data, "AYY", "06580008",&tcmInfo.accCal.AYY)) { return; }
            if (check_send_next(data, "AYZ", "06600008",&tcmInfo.accCal.AYZ)) { return; }
            if (check_send_next(data, "AZX", "06680008",&tcmInfo.accCal.AZX)) { return; }
            if (check_send_next(data, "AZY", "06700008",&tcmInfo.accCal.AZY)) { return; }
            if (check_send_next(data, "AZZ", "06780008",&tcmInfo.accCal.AZZ)) { return; }
            if (check_send_next(data, "AXV", "06800008",&tcmInfo.accCal.AXV)) { return; }
            if (check_send_next(data, "AYV", "06880008",&tcmInfo.accCal.AYV)) { return; }
            if (check_send_next(data, "AZV", "06900008",&tcmInfo.accCal.AZV)) { return; }
            if (check_send_next(data, "AXC", "06980008",&tcmInfo.accCal.AXC)) { return; }
            if (check_send_next(data, "AYC", "06A00008",&tcmInfo.accCal.AYC)) { return; }
            if (check_send_next(data, "AZC", "06A80008",&tcmInfo.accCal.AZC)) { return; }
            if (check_send_next(data, "MXX", "06B00008",&tcmInfo.magCal.MXX)) { return; }
            if (check_send_next(data, "MXY", "06B80008",&tcmInfo.magCal.MXY)) { return; }
            if (check_send_next(data, "MXZ", "06C00008",&tcmInfo.magCal.MXZ)) { return; }
            if (check_send_next(data, "MYX", "06C80008",&tcmInfo.magCal.MYX)) { return; }
            if (check_send_next(data, "MYY", "06D00008",&tcmInfo.magCal.MYY)) { return; }
            if (check_send_next(data, "MYZ", "06D80008",&tcmInfo.magCal.MYZ)) { return; }
            if (check_send_next(data, "MZX", "06E00008",&tcmInfo.magCal.MZX)) { return; }
            if (check_send_next(data, "MZY", "06E80008",&tcmInfo.magCal.MZY)) { return; }
            if (check_send_next(data, "MZZ", "06F00008",&tcmInfo.magCal.MZZ)) { return; }
            if (check_send_next(data, "MXV", "06F80008",&tcmInfo.magCal.MXV)) { return; }
            if (check_send_next(data, "MYV", "06000108",&tcmInfo.magCal.MYV)) { return; }
            if (check_send_next(data, "MZV", "06080108",NULL)) { return; }
            if (check_send_next(data, "MRF", "06100108",NULL)) { return; }
            if (check_send_next(data, "TMX", "06180108",NULL)) { return; }
            if (check_send_next(data, "TMY", "06200108",NULL)) { return; }
            if (check_send_next(data, "TMZ", "06280108",NULL)) { return; }
            if (check_send_next(data, "TMX", "06300108",NULL)) { return; }
            if (check_send_next(data, "TMY", "06380108",NULL)) { return; }
            if (check_send_next(data, "TMZ", "06400108",NULL)) { return; }
            if (check_send_next(data, "HSE", "06480108", NULL))
            {
                get_calibrations = false;
                ESP_LOGI(TAG, "All calibrations parsed");
                return;
            }
        }
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
        ESP_LOGE(TAG, "No USB devices found");
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

    /* USED FOR SINGLE COMMAND TESTING */
	bool send_once = false;
	bool sent_once = false;

    while (1) {
		bool sent_first_calib = false;
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
            if (send_once) {
                if (!sent_once) {
                    send_command(RESET_CMD);
                    sent_once = true;
                }
            }
			else if (get_version){
				send_command(FIRMWARE_VERSION_CMD);
			}
			else if (get_serialNum) {
				send_command(SERIAL_NUMBER_CMD);
			}
			else if (get_calibrations) {
				if (!sent_first_calib) {
					char calib_cmd[32];
					snprintf(calib_cmd, sizeof(calib_cmd), "%s 0600008", CALIBRATION_CMD);
					send_command(calib_cmd);
					sent_first_calib = true;
				}
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
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
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

    while (!tcm_connected) {
        ESP_LOGV(TAG, "Waiting for TCM to connect...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "TCM connected");
}
