#pragma once

#include <stdint.h>
#include <stdbool.h>

void runTcm(void);
void initTcm(void);

typedef struct {
    float x;
    float y;
    float z;
} XYZ;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} rawXYZ;

typedef struct {
    float gain[3][3];   // 3x3 gain matrix
    float offset[3];    // 3x1 offset vector
    float cubic[3];     // 3x1 cubic correction
	float AXX;
	float AXY;
	float AXZ;
	float AYX;
	float AYY;
	float AYZ;
	float AZX;
	float AZY;
	float AZZ;
    float AXV;
	float AYV;
	float AZV;
    float AXC;
	float AYC;
	float AZC;
} CubicAccelerometer;

typedef struct {
    float softIron[3][3];  // 3x3 soft-iron correction matrix
    float hardIron[3];     // 3x1 hard-iron offset
	float MXX;
	float MXY;
	float MXZ;
	float MYX;
	float MYY;
	float MYZ;
	float MZX;
	float MZY;
	float MZZ;
	float MXV;
	float MYV;
    float MZV;
} CubicMagnetometer;

typedef struct {
    XYZ acc;
    XYZ mag;
    float temp;
    float batt;
} Sensors;

typedef struct {
    rawXYZ acc;
    rawXYZ mag;
    uint16_t temp;
    uint16_t batt;
} rawSensors;

typedef struct {
    float rollRad;
    float pitchRad;
    float yawRad;
} RPY;

typedef struct {
    float north;
    float east;
} Velocity;

typedef struct {
    float TMO;
    float TMR;
    float TMA;
    float TMB;
    float TMC;
    float TMD;
} TempCalCoef;

typedef struct {
    char version[12];
    char serialNum[12];
    rawSensors raw;
    Sensors scaled;
    TempCalCoef tempCal;
    CubicAccelerometer accCal;
    CubicMagnetometer magCal;
    RPY orientation;
    float headingDeg;
    Velocity current;
} TcmInfo;

typedef struct {
    Sensors rawSum;
    int sampleCount;
} TcmAverage;

extern TcmInfo tcmInfo;
