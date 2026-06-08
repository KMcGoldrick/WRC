// ------------------------------
// USB and Sensor Functions
// ------------------------------
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "myTcm.h"

// Initialize UART for RS485
void init_rs485(void);

// Send bytes over UART RS485
void send_rs485_bytes(const uint8_t* data, size_t len);

// Send text over UART RS485
void send_rs485_text(const char* text);

// Initialize USB host
extern bool initUsbHostMode(int log_level);

// Milliseconds since boot
extern unsigned long millis(void);

// Connect a USB device
extern bool connectDeviceUsb(int vid, int pid);

// Get raw sensor data from a TCM via USB
extern bool getSensorsRawUSB(rawSensors* out_sensors, const char* command);

// Get string response from USB device
extern bool getStrUsb(char* save_as, size_t save_size, const char* command);

// Get float from ASCII85 encoded response via USB
extern bool getFloatAscii85Usb(float* out_value, const char* item, const char* command, const char* address);

bool getSensorsRawUsb(rawSensors* out_sensors, const char* command);

int rs485_available();
int read_rs485_bytes(uint8_t* buffer, int max_len, int timeout_ms);
int read_rs485_line(char* buffer, int max_len, int timeout_ms);
void rs485_receive_task(void* arg);
esp_err_t send_commandUsb(const char* cmd);

