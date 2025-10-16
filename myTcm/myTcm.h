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
    float gain[3][3];   // 3x3 gain matrix
    float offset[3];    // 3x1 offset vector
    float cubic[3];     // 3x1 cubic correction
} CubicAccelerometer;

typedef struct {
    float softIron[3][3];  // 3x3 soft-iron correction matrix
    float hardIron[3];     // 3x1 hard-iron offset
} CubicMagnetometer;

typedef struct {
    XYZ acc;
    XYZ mag;
    float temp;
    float batt;
} Sensors;

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
    int TMO;
    int TMR;
    float TMA;
    float TMB;
    float TMC;
    float TMD;
} TempCalCoef;

typedef struct {
    char version[12];
    char serialNum[12];
    Sensors raw;
    Sensors scaled;
    TempCalCoef tempCal;
    CubicMagnetometer magCal;
    CubicAccelerometer accCal;
    RPY orientation;
    float headingDeg;
    Velocity current;
} TcmInfo;

typedef struct {
    Sensors rawSum;
    int sampleCount;
} TcmAverage;

extern TcmInfo tcmInfo;
