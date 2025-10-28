// ------------------------------
// USB and Sensor Functions
// ------------------------------
#pragma once
#include "myTCM.h"
extern bool initUsb(void);
extern unsigned long millis();
extern bool connectDevice(int vid, int pid);
extern bool getSensorsRawUSB(rawSensors* out_sensors, const char* command);
extern bool getStrUsb(char* save_as, size_t save_size, const char* command);
extern bool getFloatAscii85Usb(float* out_value, const char* item, const char* command, const char* address);
