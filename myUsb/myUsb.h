// ------------------------------
// USB and Sensor Functions
// ------------------------------
#pragma once
#include "myTCM.h"
extern bool initUsb(void);
extern bool readSensors(void);
extern bool sensorsReady(void);
extern bool calibrationsReady(void);
extern bool connectDevice(int vid, int pid);
extern bool getFloatUsb(float* out_value, const char* command);
extern bool getSensorsRawUSB(rawSensors* out_sensors, const char* command);
extern bool getStrUsb(char* save_as, size_t save_size, const char* command);
extern bool getFloatAscii85Usb(float* out_value, const char* command, const char* address);
