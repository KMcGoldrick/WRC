// ------------------------------
// Standard C Library
// ------------------------------
#include <math.h>
#include <stdio.h>
#include <string.h>

// ------------------------------
// ESP-IDF Headers
// ------------------------------
#include "esp_log.h"

// ------------------------------
// Project Headers
// ------------------------------
#include "myNvs.h"
#include "myTcm.h"
#include "myUsb.h"
#include "WRCDefs.h"

#define TAG "myTcm"

// Global TCM info
TcmInfo tcmInfo;
TcmAverage tcmAvg;

// ------------------------------
// Placeholder: Calibration values
// ------------------------------
bool getCalibrations(void) {
    ESP_LOGE(TAG, "Get calibrations not implemented");

    tcmInfo.tempCal = (TempCalCoef){ 0, 10000, 0.0011238100354f, 0.0002349457073f, 0.0000000848361f, 0.0f };
    tcmInfo.accCal = (CubicAccelerometer){
        .gain = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} },
        .offset = { 0.0f, 0.0f, 0.0f },
        .cubic = { 0.0f, 0.0f, 0.0f }
    };
    tcmInfo.magCal = (CubicMagnetometer){
        .softIron = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} },
        .hardIron = { 0.0f, 0.0f, 0.0f }
    };
    return true;
}

// ------------------------------
// Placeholder: Raw sensor readings
// ------------------------------
bool getRaws(void) {
    ESP_LOGE(TAG, "Get raw sensor data not implemented");

    tcmInfo.raw.batt = 3700;      // mV
    tcmInfo.raw.temp = 2500.0f;   // Raw temperature
    tcmInfo.raw.acc = (XYZ){ 512.0f, 0.0f, -512.0f };
    tcmInfo.raw.mag = (XYZ){ 100.0f, 200.0f, -50.0f };

    return true;
}

// ------------------------------
// Temperature and Battery
// ------------------------------
float calcTempC(void) {
    if (tcmInfo.tempCal.TMR == 0) {
        ESP_LOGE(TAG, "Calibration error: TMR is zero");
        return ZERO_KELVIN;
    }

    if (tcmInfo.raw.temp < 0.0f || tcmInfo.raw.temp >= MAX_INT16) {
        ESP_LOGE(TAG, "Raw temperature out of range: %.2f", tcmInfo.raw.temp);
        return ZERO_KELVIN;
    }

    float denom_temp = MAX_INT16 - tcmInfo.raw.temp;
    if (denom_temp == 0.0f) {
        ESP_LOGE(TAG, "Division by zero in temperature calculation");
        return ZERO_KELVIN;
    }

    float temp = (tcmInfo.raw.temp * tcmInfo.tempCal.TMR) / denom_temp;
    if (temp <= 0.0f) {
        ESP_LOGE(TAG, "Log of non-positive value: temp=%.6f", temp);
        return ZERO_KELVIN;
    }

    float log_temp = logf(temp);
    float denom = tcmInfo.tempCal.TMA +
        tcmInfo.tempCal.TMB * log_temp +
        tcmInfo.tempCal.TMD * log_temp * log_temp +
        tcmInfo.tempCal.TMC * log_temp * log_temp * log_temp;

    if (denom == 0.0f) {
        ESP_LOGE(TAG, "Final denominator is zero in temperature calculation");
        return ZERO_KELVIN;
    }

    return 1.0f / denom + ZERO_KELVIN;
}

float calcBattV(void) {
    return tcmInfo.raw.batt / 1000.0f; // Convert mV to V
}

// ------------------------------
// Accelerometer and Magnetometer
// ------------------------------
XYZ calcAcc(void) {
    float raw[3] = { tcmInfo.raw.acc.x / 1024.0f, tcmInfo.raw.acc.y / 1024.0f, tcmInfo.raw.acc.z / 1024.0f };
    XYZ acc = { 0 };

    // Apply gain matrix
    for (int i = 0; i < 3; i++) {
        acc.x += tcmInfo.accCal.gain[0][i] * raw[i];
        acc.y += tcmInfo.accCal.gain[1][i] * raw[i];
        acc.z += tcmInfo.accCal.gain[2][i] * raw[i];
    }

    // Apply offset and cubic correction
    acc.x += tcmInfo.accCal.offset[0] + tcmInfo.accCal.cubic[0] * powf(raw[0], 3);
    acc.y += tcmInfo.accCal.offset[1] + tcmInfo.accCal.cubic[1] * powf(raw[1], 3);
    acc.z += tcmInfo.accCal.offset[2] + tcmInfo.accCal.cubic[2] * powf(raw[2], 3);

    return acc;
}

// ------------------------------
// Magnetometer Calibration
// ------------------------------
XYZ calcMag(void) {
    float raw[3] = { tcmInfo.raw.mag.x, tcmInfo.raw.mag.y, tcmInfo.raw.mag.z };
    float corrected[3], calibrated[3];

    // Step 1: apply hard-iron offset
    for (int i = 0; i < 3; i++) {
        corrected[i] = raw[i] + tcmInfo.magCal.hardIron[i];
    }

    // Step 2: apply soft-iron correction matrix
    for (int i = 0; i < 3; i++) {
        calibrated[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            calibrated[i] += tcmInfo.magCal.softIron[i][j] * corrected[j];
        }
    }

    // Step 3: store corrected values
    return (XYZ) { calibrated[0], calibrated[1], calibrated[2] };
}

// ------------------------------
// Roll-Pitch-Yaw Calculation
// ------------------------------
RPY calcRPY(void) {
    RPY rpy;

    // Roll: atan2(acc.y, acc.z)
    rpy.rollRad = atan2f(tcmInfo.scaled.acc.y, tcmInfo.scaled.acc.z);

    // Pitch: atan2(-acc.x, acc.y*sin(roll) + acc.z*cos(roll))
    rpy.pitchRad = atan2f(
        -tcmInfo.scaled.acc.x,
        tcmInfo.scaled.acc.y * sinf(rpy.rollRad) + tcmInfo.scaled.acc.z * cosf(rpy.rollRad)
    );

    // Magnetometer compensation
    float by = tcmInfo.scaled.mag.z * sinf(rpy.rollRad) - tcmInfo.scaled.mag.y * cosf(rpy.rollRad);
    float bx = tcmInfo.scaled.mag.x * cosf(rpy.pitchRad)
        + tcmInfo.scaled.mag.y * sinf(rpy.pitchRad) * sinf(rpy.rollRad)
        + tcmInfo.scaled.mag.z * sinf(rpy.pitchRad) * cosf(rpy.rollRad);

    // Yaw: atan2(by, bx)
    rpy.yawRad = atan2f(by, bx);

    return rpy;
}

float calcHeading(void) {
    float heading = tcmInfo.orientation.yawRad * 180.0f / M_PI;
    heading = fmodf(heading + 180.0f + DECLINATION_DEG, 360.0f);
    if (heading < 0.0f) heading += 360.0f;
    return heading - 180.0f;
}

// ------------------------------
// Velocity (Placeholder)
// ------------------------------
float speedFromTilt(void) {
    ESP_LOGE(TAG, "Speed from tilt not implemented");
    return 24.7f;
}

Velocity calcCurrent(void) {
    float speed = speedFromTilt();
    Velocity vel;
    vel.north = speed * cosf(tcmInfo.headingDeg * M_PI / 180.0f);
    vel.east = speed * sinf(tcmInfo.headingDeg * M_PI / 180.0f);
    return vel;
}

// ------------------------------
// Main TCM Calculation
// ------------------------------
void calcTcm(void) {
    tcmInfo.scaled.batt = calcBattV();
    tcmInfo.scaled.temp = calcTempC();
    tcmInfo.scaled.acc = calcAcc();
    tcmInfo.scaled.mag = calcMag();
    tcmInfo.orientation = calcRPY();
    tcmInfo.headingDeg = calcHeading();
    tcmInfo.current = calcCurrent();
}

// ------------------------------
// Averaging
// ------------------------------
void addRaws(void) {
    tcmAvg.rawSum.acc.x += tcmInfo.raw.acc.x;
    tcmAvg.rawSum.acc.y += tcmInfo.raw.acc.y;
    tcmAvg.rawSum.acc.z += tcmInfo.raw.acc.z;

    tcmAvg.rawSum.mag.x += tcmInfo.raw.mag.x;
    tcmAvg.rawSum.mag.y += tcmInfo.raw.mag.y;
    tcmAvg.rawSum.mag.z += tcmInfo.raw.mag.z;

    tcmAvg.rawSum.temp += tcmInfo.raw.temp;
    tcmAvg.rawSum.batt += tcmInfo.raw.batt;
}

void calcAndCopyAverages(void) {
    tcmInfo.raw.acc.x = tcmAvg.rawSum.acc.x / tcmAvg.sampleCount;
    tcmInfo.raw.acc.y = tcmAvg.rawSum.acc.y / tcmAvg.sampleCount;
    tcmInfo.raw.acc.z = tcmAvg.rawSum.acc.z / tcmAvg.sampleCount;

    tcmInfo.raw.mag.x = tcmAvg.rawSum.mag.x / tcmAvg.sampleCount;
    tcmInfo.raw.mag.y = tcmAvg.rawSum.mag.y / tcmAvg.sampleCount;
    tcmInfo.raw.mag.z = tcmAvg.rawSum.mag.z / tcmAvg.sampleCount;

    tcmInfo.raw.temp = tcmAvg.rawSum.temp / tcmAvg.sampleCount;
    tcmInfo.raw.batt = tcmAvg.rawSum.batt / tcmAvg.sampleCount;
}

void resetAverages(void) {
    tcmAvg.rawSum = (Sensors){ {0,0,0}, {0,0,0}, 0.0f, 0.0f };
    tcmAvg.sampleCount = 0;
}

// ------------------------------
// TCM Algorithm
// ------------------------------
void dispTcm() {
    ESP_LOGI(TAG, "TCM Version: %s", tcmInfo.version);
    ESP_LOGI(TAG, "Serial Number: %s", tcmInfo.serialNum);
    ESP_LOGI(TAG, "Battery: %.2f V", tcmInfo.scaled.batt);
    ESP_LOGI(TAG, "Temperature: %.2f C", tcmInfo.scaled.temp);
    ESP_LOGI(TAG, "Acceleration (g): X=%.2f Y=%.2f Z=%.2f",
        tcmInfo.scaled.acc.x, tcmInfo.scaled.acc.y, tcmInfo.scaled.acc.z);
    ESP_LOGI(TAG, "Magnetometer (mG): X=%.2f Y=%.2f Z=%.2f",
        tcmInfo.scaled.mag.x, tcmInfo.scaled.mag.y, tcmInfo.scaled.mag.z);
    ESP_LOGI(TAG, "Orientation (rad): Roll=%.2f Pitch=%.2f Yaw=%.2f",
        tcmInfo.orientation.rollRad, tcmInfo.orientation.pitchRad, tcmInfo.orientation.yawRad);
    ESP_LOGI(TAG, "Heading (deg): %.2f", tcmInfo.headingDeg);
    ESP_LOGI(TAG, "Current Velocity: North=%.2f East=%.2f",
        tcmInfo.current.north, tcmInfo.current.east);
}

void tcmAlgo(void) {
    if (tcmAvg.sampleCount < NUM_ITERATIONS_TO_AVERAGE) {
        ESP_LOGI(TAG, "Adding sample %d of %d", tcmAvg.sampleCount, NUM_ITERATIONS_TO_AVERAGE);
        addRaws();
        tcmAvg.sampleCount++;
    }

    if (tcmAvg.sampleCount == NUM_ITERATIONS_TO_AVERAGE) {
        ESP_LOGI(TAG, "Averaged %d samples", NUM_ITERATIONS_TO_AVERAGE);
        calcAndCopyAverages();
        calcTcm();
        dispTcm();
        resetAverages();
    }
}

// ------------------------------
// Initialization and Run
// ------------------------------
void initTcm(void) {
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
    esp_log_level_set(TAG, ESP_LOG_INFO);
    resetAverages();
    getCalibrations();
	readSensors();
    ESP_LOGI(TAG, "TCM Initialized");
}

void runTcm(void) 
{
    if (!areSensorsReady()) {
        ESP_LOGI(TAG, "Sensors are not ready");
        return;
    }
    calcTcm();
    dispTcm();
    tcmAlgo();
	readSensors();
}
