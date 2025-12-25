// ------------------------------
// USB and Sensor Functions
// ------------------------------
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "myTcm.h"

// Initialize USB host
extern bool initUsb(int log_level);

// Milliseconds since boot
extern unsigned long millisUsb(void);

// Connect a USB device
extern bool connectDeviceUsb(int vid, int pid);

// Get raw sensor data from a TCM via USB
extern bool getSensorsRawUSB(rawSensors* out_sensors, const char* command);

// Get string response from USB device
extern bool getStrUsb(char* save_as, size_t save_size, const char* command);

// Get float from ASCII85 encoded response via USB
extern bool getFloatAscii85Usb(float* out_value, const char* item, const char* command, const char* address);

bool getSensorsRawUsb(rawSensors* out_sensors, const char* command);
