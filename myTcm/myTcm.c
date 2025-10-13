#include <math.h>
#include <stdio.h>           
#include <string.h>          
#include <stdlib.h>
#include "esp_log.h"         
#include "nvs_flash.h"       
#include "nvs.h"             
#include "myNvs.h"           
#include "myTcm.h"
#include "WRCDefs.h"  

// Questions for Lowell Instruments:
// 
// 

// Lowell Instruments TCM Commands
#define FIRMWARE_VERSION_CMD   "GFV"
#define CALIBRATION_CMD        "RHS"
#define INTERVAL_TIME_CMD      "GIT"
#define LOGGER_INFO_CMD        "RLI"
#define LOGGER_SETTINGS_CMD    "GLS"
#define PAGE_COUNT_CMD         "GPC"
#define RESET_CMD              "RST"
#define RUN_CMD                "RUN"
#define SD_CAPACITY_CMD        "CTS"
#define SD_FILE_SIZE_CMD       "FSZ"
#define SD_FREE_SPACE_CMD      "CFS"
#define SENSOR_READINGS_CMD    "GSR"
#define SERIAL_NUMBER_CMD      "GSN"
#define START_TIME_CMD         "GST"
#define STATUS_CMD             "STS"
#define STOP_CMD               "STP"
#define SWS_CMD                "SWS"
#define RWS_CMD                "RWS"
#define SET_TIME_CMD           "STM"
#define TIME_CMD               "GTM"
#define DEL_FILE_CMD           "DEL"
#define LOGGER_INFO_CMD_W      "WLI"
#define LOGGER_HSA_CMD_W       "WHS"
#define REQ_FILE_NAME_CMD      "RFN"
#define DIR_CMD                "DIR"



#define DECLINATION_DEG  -7.66f  // Magnetic declination for Grenville NC

#define NUM_ITERATIONS 20

TcmInfo tcmInfo;
TcmAverage tcmAvg;

bool getCalibrations(void) {
    ESP_LOGE("TCM", "Get calibrations not implemented");
    // Default to hardcoded calibration values
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

bool getStringCmd(const char* cmd, char* response, size_t responseSize) {
	ESP_LOGE("TCM", "Get string command not implemented");
    return false;
}

bool getRaws() {
    ESP_LOGE("TCM", "Get raw sensor data not implemented");
	tcmInfo.raw.Batt = 3700; // Fix Example raw battery value in mV
	tcmInfo.raw.Temp = 2500.0f; // Fix Example raw temperature value
	tcmInfo.raw.Acc = (XYZ){ 512.0f, 0.0f, -512.0f }; // Fix Example raw accelerometer values
	tcmInfo.raw.Mag = (XYZ){ 100.0f, 200.0f, -50.0f }; // Fix Example raw magnetometer values
    return true;
}

bool getVersion(void) {
    if (getStringCmd(FIRMWARE_VERSION_CMD, tcmInfo.version, sizeof(tcmInfo.version))) {
        return true;
    }
    else {
        ESP_LOGE("TCM", "Failed to get TCM version");
        strncpy(tcmInfo.version, "?.?.?", sizeof(tcmInfo.version) - 1);
        tcmInfo.version[sizeof(tcmInfo.version) - 1] = '\0';
        return false;
    }
}

float calcTempC() {
    float temp, log_temp, denom, result;

    // Protect against invalid calibration coefficients
    if (tcmInfo.tempCal.TMR == 0) {
        ESP_LOGE("TCM", "Calibration error: TMR is zero");
        return ZERO_KELVIN;
    }

    // Protect against raw.Temp out of range
    if (tcmInfo.raw.Temp < 0.0f || tcmInfo.raw.Temp >= MAX_INT16) {
        ESP_LOGE("TCM", "Raw temperature out of range: %.2f", tcmInfo.raw.Temp);
        return ZERO_KELVIN;
    }

    // Prevent division by zero
    float denom_temp = MAX_INT16 - tcmInfo.raw.Temp;
    if (denom_temp == 0.0f) {
        ESP_LOGE("TCM", "Division by zero in temperature calculation");
        return ZERO_KELVIN;
    }

    temp = (tcmInfo.raw.Temp * tcmInfo.tempCal.TMR) / denom_temp;

    // Prevent log of non-positive value
    if (temp <= 0.0f) {
        ESP_LOGE("TCM", "Logarithm of non-positive value in temperature calculation: temp=%.6f", temp);
        return ZERO_KELVIN;
    }

    log_temp = logf(temp);

    denom = tcmInfo.tempCal.TMA +
        tcmInfo.tempCal.TMB * log_temp +
        tcmInfo.tempCal.TMD * log_temp * log_temp +
        tcmInfo.tempCal.TMC * log_temp * log_temp * log_temp;

    if (denom == 0.0f) {
        ESP_LOGE("TCM", "Final denominator is zero in temperature calculation");
        return ZERO_KELVIN;
    }

    result = 1.0f / denom + ZERO_KELVIN;
    return result;
}

float calcBattV() {
	float result;
	result = (tcmInfo.raw.Batt / 1000.0f); // Convert mV to V
    return result;
}

XYZ calcAcc() {
    float raw_accel[3];
    XYZ acc;

    // Step 1: scale raw readings (like raw_meter / 1024.0)
    raw_accel[0] = tcmInfo.raw.Acc.X / 1024.0f;
    raw_accel[1] = tcmInfo.raw.Acc.Y / 1024.0f;
    raw_accel[2] = tcmInfo.raw.Acc.Z / 1024.0f;


    // Step 2: apply gain matrix (dot product)
    for (int i = 0; i < 3; i++) {
        acc.X = 0.0f;
        for (int j = 0; j < 3; j++) {
            acc.X += tcmInfo.accCal.gain[i][j] * raw_accel[j];
        }
    }
    for (int i = 0; i < 3; i++) {
        acc.Y = 0.0f;
        for (int j = 0; j < 3; j++) {
            acc.Y += tcmInfo.accCal.gain[i][j] * raw_accel[j];
        }
    }
    for (int i = 0; i < 3; i++) {
        acc.Z = 0.0f;
        for (int j = 0; j < 3; j++) {
            acc.Z += tcmInfo.accCal.gain[i][j] * raw_accel[j];
        }
    }

    // Step 3: add offset and cubic correction
    for (int i = 0; i < 3; i++) {
        acc.X += tcmInfo.accCal.offset[i] + tcmInfo.accCal.cubic[i] * powf(raw_accel[i], 3);
        acc.Y += tcmInfo.accCal.offset[i] + tcmInfo.accCal.cubic[i] * powf(raw_accel[i], 3);
        acc.Z += tcmInfo.accCal.offset[i] + tcmInfo.accCal.cubic[i] * powf(raw_accel[i], 3);
    }

	return acc;
}

XYZ calcMag() {
    float raw_mag[3];
    float corrected[3];
    float calibrated[3];

    // Step 1: load raw magnetometer readings
    raw_mag[0] = tcmInfo.raw.Mag.X;
    raw_mag[1] = tcmInfo.raw.Mag.Y;
    raw_mag[2] = tcmInfo.raw.Mag.Z;

    // Step 2: apply hard-iron offset (bias correction)
    for (int i = 0; i < 3; i++) {
        corrected[i] = raw_mag[i] + tcmInfo.magCal.hardIron[i];
    }

    // Step 3: apply soft-iron matrix (3x3 multiply)
    for (int i = 0; i < 3; i++) {
        calibrated[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            calibrated[i] += tcmInfo.magCal.softIron[i][j] * corrected[j];
        }
    }

    // Step 4: store result
    XYZ mag = { calibrated[0], calibrated[1], calibrated[2] };
    return mag;
}

RPY calcRPY() {
	RPY rpy;
    // Roll: atan2(accel.Y, accel.Z)
    rpy.rollRad = atan2f(tcmInfo.scaled.Acc.Y, tcmInfo.scaled.Acc.Z);

    // Pitch: atan2(-accel.X, accel.Y * sin(roll) + accel.Z * cos(roll))
    rpy.pitchRad = atan2f(-tcmInfo.scaled.Acc.X,
        tcmInfo.scaled.Acc.Y * sinf(rpy.rollRad) + tcmInfo.scaled.Acc.Z * cosf(rpy.rollRad));

    // by = mag.Z * sin(roll) - mag.Y * cos(roll)
    float by = tcmInfo.scaled.Mag.Z * sinf(rpy.rollRad) - tcmInfo.scaled.Mag.Y * cosf(rpy.rollRad);

    // bx = mag.X * cos(pitch) + mag.Y * sin(pitch) * sin(roll) + mag.Z * sin(pitch) * cos(roll)
    float bx = tcmInfo.scaled.Mag.X * cosf(rpy.pitchRad)
        + tcmInfo.scaled.Mag.Y * sinf(rpy.pitchRad) * sinf(rpy.rollRad)
        + tcmInfo.scaled.Mag.Z * sinf(rpy.pitchRad) * cosf(rpy.rollRad);

    // Yaw: atan2(by, bx)
    rpy.yawRad = atan2f(by, bx);

	return rpy;
}

float calcHeading() {
	float result;

    result = tcmInfo.orientation.yawRad * 180 / M_PI;  // Convert to degrees
    result = fmodf(result + 180.0f + DECLINATION_DEG, 360.0f);
    if (result < 0.0f) result += 360.0f;
    result -= 180.0f;

    return result;
}

float speedFromTilt() {
    ESP_LOGE("TCM", "Speed from tilt not implemented");
    float result;

    result = 24.7;
    return result;
}

Velocity calcCurrent() {
    float speed;
    Velocity result;

    speed = speedFromTilt();
    result.North = speed * cosf(tcmInfo.headingDeg*M_PI/180.0f);
    result.East = speed * sinf(tcmInfo.headingDeg*M_PI/180.0f);

    return result;
}

void calcTcm() {
    tcmInfo.scaled.Batt = calcBattV();
    tcmInfo.scaled.Temp = calcTempC();
    tcmInfo.scaled.Acc = calcAcc();
    tcmInfo.scaled.Mag = calcMag();
    tcmInfo.orientation = calcRPY();
    tcmInfo.headingDeg = calcHeading();
    tcmInfo.current = calcCurrent();
}

void dispTcm() {
    ESP_LOGI("TCM", "TCM Version: %s", tcmInfo.version);
    ESP_LOGI("TCM", "Battery: %.2f V", tcmInfo.scaled.Batt);
    ESP_LOGI("TCM", "Temperature: %.2f C", tcmInfo.scaled.Temp);
    ESP_LOGI("TCM", "Acceleration(g): X=%.2f Y=%.2f Z=%.2f", 
        tcmInfo.scaled.Acc.X, tcmInfo.scaled.Acc.Y, tcmInfo.scaled.Acc.Z);
    ESP_LOGI("TCM", "Magnetometer(mG): X=%.2f Y=%.2f Z=%.2f", 
        tcmInfo.scaled.Mag.X, tcmInfo.scaled.Mag.Y, tcmInfo.scaled.Mag.Z);
    ESP_LOGI("TCM", "Orientation(rad): Roll=%.2f Pitch=%.2f Yaw=%.2f", 
        tcmInfo.orientation.rollRad, tcmInfo.orientation.pitchRad, tcmInfo.orientation.yawRad);
    ESP_LOGI("TCM", "Heading(deg): %.2f", tcmInfo.headingDeg);
    ESP_LOGI("TCM", "Current Velocity(?): North=%.2f East=%.2f", 
		tcmInfo.current.North, tcmInfo.current.East);
}

void addRaws() {
    tcmAvg.rawSum.Acc.X += tcmInfo.raw.Acc.X;
    tcmAvg.rawSum.Acc.Y += tcmInfo.raw.Acc.Y;
    tcmAvg.rawSum.Acc.Z += tcmInfo.raw.Acc.Z;
    tcmAvg.rawSum.Mag.X += tcmInfo.raw.Mag.X;
    tcmAvg.rawSum.Mag.Y += tcmInfo.raw.Mag.Y;
    tcmAvg.rawSum.Mag.Z += tcmInfo.raw.Mag.Z;
    tcmAvg.rawSum.Temp += tcmInfo.raw.Temp;
    tcmAvg.rawSum.Batt += tcmInfo.raw.Batt;
}

void calcAndCopyAverages() {
	// Calculate averages and copy to tcmInfo
    tcmInfo.raw.Acc.X = tcmAvg.rawSum.Acc.X / tcmAvg.sampleCount;
    tcmInfo.raw.Acc.Y = tcmAvg.rawSum.Acc.Y / tcmAvg.sampleCount;
    tcmInfo.raw.Acc.Z = tcmAvg.rawSum.Acc.Z / tcmAvg.sampleCount;
    tcmInfo.raw.Mag.X = tcmAvg.rawSum.Mag.X / tcmAvg.sampleCount;
    tcmInfo.raw.Mag.Y = tcmAvg.rawSum.Mag.Y / tcmAvg.sampleCount;
    tcmInfo.raw.Mag.Z = tcmAvg.rawSum.Mag.Z / tcmAvg.sampleCount;
    tcmInfo.raw.Temp = tcmAvg.rawSum.Temp / tcmAvg.sampleCount;
    tcmInfo.raw.Batt = tcmAvg.rawSum.Batt / tcmAvg.sampleCount;
}

void resetAverges() {
    tcmAvg.rawSum.Acc = (XYZ){0.0f, 0.0f, 0.0f};
    tcmAvg.rawSum.Mag = (XYZ){0.0f, 0.0f, 0.0f};
    tcmAvg.rawSum.Temp = 0.0f;
    tcmAvg.rawSum.Batt = 0.0f;
    tcmAvg.sampleCount = 0;
}

void tcmAlgo() {
    /*
      Due to vortex induced oscillations and turbulence,
      we recommend taking 8 samples per second for at least 20 seconds
      and averaging the accelerometer and magnetometer values
      and converting to one speed and direction measurement.
      That is faster than you will be able to sample over the virtual comm port,
      but you can sample at a slower rate for a longer time and get a similar result.
    */
    if (tcmAvg.sampleCount < NUM_ITERATIONS) {
        ESP_LOGI("TCM", "Adding sample %d of %d", tcmAvg.sampleCount, NUM_ITERATIONS);
        addRaws();
        tcmAvg.sampleCount++;
    }
    if (tcmAvg.sampleCount == NUM_ITERATIONS) {
        ESP_LOGI("TCM", "Averaged %d samples", NUM_ITERATIONS);
		calcAndCopyAverages();
        // Recalculate values based on averaged raw values
        calcTcm();
        dispTcm();
        // Reset for next averaging cycle
        resetAverges();
        return;
	}
}

void initTcm() {
	getVersion();
    getCalibrations();

    tcmAvg = (TcmAverage){
        .rawSum.Acc = {0.0f, 0.0f, 0.0f},
        .rawSum.Mag = {0.0f, 0.0f, 0.0f},
        .rawSum.Temp = 0.0f,
		.rawSum.Batt = 0.0f,
        .sampleCount = 0
	};

	ESP_LOGI("TCM", "TCM Version: %s", tcmInfo.version);
	ESP_LOGI("TCM", "TCM Initialized");
}

void runTcm() {
    if (!getRaws()) {
        ESP_LOGE("TCM", "Failed to get raw sensor data");
        return;
    }
    else {
        calcTcm();
		dispTcm();
		tcmAlgo();
    }
}

