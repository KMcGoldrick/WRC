// ------------------------------
// USB and Sensor Functions
// ------------------------------
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "myTcm.h"

// Initialize USB host
extern bool initUsb(void);

// Milliseconds since boot
extern unsigned long millis(void);

// Connect a USB device
// is_tcm: true = TCM device, false = logging device
extern bool connectDevice(int vid, int pid, bool is_tcm);

// Get raw sensor data from a TCM via USB
extern bool getSensorsRawUSB(rawSensors* out_sensors, const char* command);

// Get string response from USB device
// is_tcm: true = TCM, false = logging
extern bool getStrUsb(bool is_tcm, char* save_as, size_t save_size, const char* command);

// Get float from ASCII85 encoded response via USB
// is_tcm: true = TCM, false = logging
extern bool getFloatAscii85Usb(bool is_tcm, float* out_value, const char* item, const char* command, const char* address);
