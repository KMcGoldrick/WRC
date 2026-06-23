#pragma once

#include <stdint.h>
#include <stdbool.h>

extern bool runTcm(bool debug, bool average, bool dataAsText, int select);
extern bool initTcm(int level, bool debug, int select, bool asText);

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
} CubicAccelerometer;

typedef struct {
    float softIron[3][3];  // 3x3 soft-iron correction matrix
    float hardIron[3];     // 3x1 hard-iron offset
    float tempRef;         // Reference temperature for compensation
    float tempSlope[3];    // Temperature compensation slopes for X, Y, Z
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
} TempCalCoef;

typedef struct {
    char version[12];          //6
    char serialNum[12];        //6
    rawSensors raw;            //3 acc 4 mag 5 temp/batt
    Sensors scaled;            //3 acc 4 mag 5 temp/batt
    TempCalCoef tempCal;       //7
    CubicAccelerometer accCal; //8 9
    CubicMagnetometer magCal;  //10 11
    RPY orientation;           //2
    float tiltRad;             //12
	float speed;               //12
    float headingDeg;          //1
    Velocity current;          //1
} TcmInfo;

typedef struct {
    Sensors rawSum;
    int sampleCount;
} TcmAverage;

extern TcmInfo tcmInfo;

typedef struct {
    float tiltDeg;   // tilt angle from vertical (degrees)
    float speed;     // corresponding water speed (m/s or cm/s — match your cal data)
} TiltPoint;
// Example calibration table — replace with real cal data from Lowell
// Tilt angles must be ascending, range [0, 90]
static const TiltPoint _tiltCurve[] = {
    {  0.0f, 0.00f },
    {  5.0f, 0.10f },
    { 10.0f, 0.22f },
    { 15.0f, 0.36f },
    { 20.0f, 0.52f },
    { 25.0f, 0.70f },
    { 30.0f, 0.90f },
    { 40.0f, 1.35f },
    { 50.0f, 1.85f },
    { 60.0f, 2.40f },
    { 90.0f, 3.00f },
};
static const int _tiltCurveLen = sizeof(_tiltCurve) / sizeof(_tiltCurve[0]);

