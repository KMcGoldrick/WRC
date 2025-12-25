// ------------------------------
// Uart and Sensor Functions
// ------------------------------
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "myTcm.h"

// Initialize Uart host
extern bool initUart(int log_level);

// Milliseconds since boot
extern unsigned long millisUart(void);

// Connect a Uart device
extern bool connectDeviceUart(int vid, int pid);

// Get raw sensor data from a TCM via Uart
extern bool getSensorsRawUart(rawSensors* out_sensors, const char* command);

// Get string response from Uart device
extern bool getStrUart(char* save_as, size_t save_size, const char* command);

// Get float from ASCII85 encoded response via Uart
extern bool getFloatAscii85Uart(float* out_value, const char* item, const char* command, const char* address);

bool getSensorsRawUart(rawSensors* out_sensors, const char* command);
