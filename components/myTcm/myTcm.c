// ------------------------------
// Standard C Library
// ------------------------------
#include <math.h>
#include <stdio.h>
#include <string.h>

// ------------------------------
// ESP-IDF System Headers
// ------------------------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ------------------------------
// Project Headers
// ------------------------------
#include "myNvs.h"
#include "myTcm.h"
#include "myUsb.h"
#include "WRCDefs.h"

#define TAG "TCM"

// Global TCM info
TcmInfo tcmInfo;
TcmAverage tcmAvg;

// ------------------------------
// Placeholder: Calibration values
// ------------------------------
bool defaultCalibrations(void) {
    tcmInfo.tempCal = (TempCalCoef){ 0.0f, 10000.0f, 0.0011238100354f, 0.0002349457073f, 0.0000000848361f };
    tcmInfo.accCal = (CubicAccelerometer){
        .gain = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} },
        .offset = { 0.0f, 0.0f, 0.0f },
        .cubic = { 0.0f, 0.0f, 0.0f }
    };
    tcmInfo.magCal = (CubicMagnetometer){
        .softIron = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} },
        .hardIron = { 0.0f, 0.0f, 0.0f }
    };
    ESP_LOGI(TAG, "Set Default Calibrations");
    return true;
}

bool defaultRaws(void) {
    tcmInfo.raw.batt = 3700;      // mV
    tcmInfo.raw.temp = 2500;      // Raw temperature
    tcmInfo.raw.acc = (rawXYZ){ 512, 0, 512 };
    tcmInfo.raw.mag = (rawXYZ){ 100, 200, -50 };
	ESP_LOGI(TAG, "Set Default Raw Sensor Values");
    return true;
}

// ------------------------------
// Temperature and Battery
// ------------------------------
float calcTempC(void)
{
    // Ensure calibration TMR is non-zero
    if (tcmInfo.tempCal.TMR == 0.0f) {
        ESP_LOGW(TAG, "Calibration error: TMR is zero");
        return DEFAULT_TEMP; // Return absolute zero on error
    }

    // Compute resistance ratio
    float raw = (float)tcmInfo.raw.temp;
    float denom_temp = (65535.0f - raw);
    if (denom_temp == 0.0f) {
        ESP_LOGW(TAG, "Division by zero in resistance calculation");
        return DEFAULT_TEMP;
    }

    float R = (raw * tcmInfo.tempCal.TMR) / denom_temp;
    if (R <= 0.0f) {
        ESP_LOGW(TAG, "Invalid resistance ratio R <= 0");
        return DEFAULT_TEMP;
    }

    float logR = logf(R);

    // Steinhart-Hart formula: 1/T = A + B*ln(R) + C*(ln(R))^3
    float denom = tcmInfo.tempCal.TMA
        + tcmInfo.tempCal.TMB * logR
        + tcmInfo.tempCal.TMC * logR * logR * logR;

    if (denom == 0.0f) {
        ESP_LOGW(TAG, "Denominator zero in temperature calculation");
        return DEFAULT_TEMP;
    }

    float tempK = 1.0f / denom;       // Temperature in Kelvin
    float tempC = tempK - 273.15f;    // Convert to °C

    return tempC;
}

float calcBattV(void) {
    return tcmInfo.raw.batt / 1000.0f; // Convert mV to V
}

// ------------------------------
// Accelerometer and Magnetometer
// ------------------------------
XYZ calcAcc(void) {
	// Convert raw accelerometer values to g's
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

XYZ calcTempCompMag(void) {
    float raw[3] = { tcmInfo.raw.mag.x, tcmInfo.raw.mag.y, tcmInfo.raw.mag.z };
    float corrected[3], calibrated[3], tempComp[3];
    float temp = tcmInfo.raw.temp;

    // Clamp temperature to calibration range [-20, 50]
    if (temp < -20.0f) temp = -20.0f;
    if (temp > 50.0f) temp = 50.0f;

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

    // Step 3: apply temperature compensation
    // M_corr = M_base + (T - T_ref) * slope
    float deltaT = temp - tcmInfo.magCal.tempRef;
    for (int i = 0; i < 3; i++) {
        tempComp[i] = calibrated[i] + deltaT * tcmInfo.magCal.tempSlope[i];
    }

    // Step 4: return final compensated result
    return (XYZ) { tempComp[0], tempComp[1], tempComp[2] };
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
    /*
		From Lowell Instruments Domino. 
        A tilt curve is a carefully calibrated lookup table that associates
        the angle of tilt with the speed of passing water.

        To load a tilt curve, instantiate the TitlCurve class with the path to
        the tilt curve file.Call the parse() method to parse the file.

        To convert tilt angle to speed, call speed_from_tilt() with the tilt angle
        (in degrees) from vertical.
    */

    ESP_LOGE(TAG, "Speed from tilt not implemented");
    return (float)(MAGIC/10.0);
}

Velocity calcCurrent(void) {
    Velocity vel;
    vel.north = tcmInfo.speed * cosf(tcmInfo.headingDeg * M_PI / 180.0f);
    vel.east = tcmInfo.speed * sinf(tcmInfo.headingDeg * M_PI / 180.0f);
    return vel;
}

// ------------------------------
// Main TCM Calculation
// ------------------------------
void calcTcm(void) {
    tcmInfo.scaled.batt = calcBattV();
    tcmInfo.scaled.temp = calcTempC();
    tcmInfo.scaled.acc = calcAcc();
    tcmInfo.scaled.mag = calcTempCompMag();
    tcmInfo.orientation = calcRPY();
    tcmInfo.speed = speedFromTilt();
    tcmInfo.headingDeg = calcHeading();
    tcmInfo.current = calcCurrent();
}

// ------------------------------
// Averaging
// ------------------------------
void addRawsToRawSum(void) {
	tcmAvg.sampleCount++;
    ESP_LOGI(TAG, "Adding sample %d of %d", tcmAvg.sampleCount, NUM_ITERATIONS_TO_AVERAGE);

    tcmAvg.rawSum.acc.x += tcmInfo.raw.acc.x;
    tcmAvg.rawSum.acc.y += tcmInfo.raw.acc.y;
    tcmAvg.rawSum.acc.z += tcmInfo.raw.acc.z;

    tcmAvg.rawSum.mag.x += tcmInfo.raw.mag.x;
    tcmAvg.rawSum.mag.y += tcmInfo.raw.mag.y;
    tcmAvg.rawSum.mag.z += tcmInfo.raw.mag.z;

    tcmAvg.rawSum.temp += tcmInfo.raw.temp;
    tcmAvg.rawSum.batt += tcmInfo.raw.batt;
}

void calcAveragesAndCopyToRaw(void) {
	// This will average the rawSum sensor readings over the number of samples collected
	// and copy the averaged values back to tcmInfo.raw
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
	ESP_LOGI(TAG, "Averages reset");
}

// ------------------------------
// TCM Algorithm
// ------------------------------
void dispTcm() {
    ESP_LOGI(TAG, "TCM Version: %s", tcmInfo.version);
    ESP_LOGI(TAG, "Serial Number: %s", tcmInfo.serialNum);
    ESP_LOGI(TAG, "Battery: %.2f V", tcmInfo.scaled.batt);
    ESP_LOGI(TAG, "Temperature: %.3f C", tcmInfo.scaled.temp);
    ESP_LOGI(TAG, "Acceleration (g): X=%.3f Y=%.3f Z=%.3f",
        tcmInfo.scaled.acc.x, tcmInfo.scaled.acc.y, tcmInfo.scaled.acc.z);
    ESP_LOGI(TAG, "Magnetometer (mG): X=%.3f Y=%.3f Z=%.3f",
        tcmInfo.scaled.mag.x, tcmInfo.scaled.mag.y, tcmInfo.scaled.mag.z);
    ESP_LOGI(TAG, "Orientation (rad): Roll=%.2f Pitch=%.2f Yaw=%.2f",
        tcmInfo.orientation.rollRad, tcmInfo.orientation.pitchRad, tcmInfo.orientation.yawRad);
	ESP_LOGI(TAG, "Speed (?): %.2f", tcmInfo.speed);
    ESP_LOGI(TAG, "Heading (deg): %.2f", tcmInfo.headingDeg);
    ESP_LOGI(TAG, "Current Velocity: North=%.2f East=%.2f",
        tcmInfo.current.north, tcmInfo.current.east);
}

void dispCalibrations(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Temperature Calibration: TMR=%f TMA=%f TMB=%f TMC=%f",
        tcmInfo.tempCal.TMR,
        tcmInfo.tempCal.TMA,
        tcmInfo.tempCal.TMB,
        tcmInfo.tempCal.TMC);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Accelerometer Calibration: Gain Matrix=[[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]]",
        tcmInfo.accCal.gain[0][0], tcmInfo.accCal.gain[0][1], tcmInfo.accCal.gain[0][2],
        tcmInfo.accCal.gain[1][0], tcmInfo.accCal.gain[1][1], tcmInfo.accCal.gain[1][2],
		tcmInfo.accCal.gain[2][0], tcmInfo.accCal.gain[2][1], tcmInfo.accCal.gain[2][2]);
    ESP_LOGI(TAG, "Accelerometer Calibration: Offsets=(%f, %f, %f)",
        tcmInfo.accCal.offset[0],
        tcmInfo.accCal.offset[1],
		tcmInfo.accCal.offset[2]);
    ESP_LOGI(TAG, "Accelerometer Calibration: Cubic=(%f, %f, %f)",
        tcmInfo.accCal.cubic[0],
		tcmInfo.accCal.cubic[1], 
		tcmInfo.accCal.cubic[2]);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Magnetometer Calibration: Soft Iron Matrix=[[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]]",
        tcmInfo.magCal.softIron[0][0], tcmInfo.magCal.softIron[0][1], tcmInfo.magCal.softIron[0][2],
		tcmInfo.magCal.softIron[1][0], tcmInfo.magCal.softIron[1][1], tcmInfo.magCal.softIron[1][2],
		tcmInfo.magCal.softIron[2][0], tcmInfo.magCal.softIron[2][1], tcmInfo.magCal.softIron[2][2]);
    ESP_LOGI(TAG, "Magnetometer Calibration: Hard Iron Offsets=(%f, %f, %f)",
        tcmInfo.magCal.hardIron[0],
		tcmInfo.magCal.hardIron[1],
		tcmInfo.magCal.hardIron[2]);
    ESP_LOGI(TAG, "Magnetometer Calibration: MRF =%f", tcmInfo.magCal.tempRef);
	ESP_LOGI(TAG, "Magnetometer Calibration: TMX =%f TMY=%f TMZ=%f", 
        tcmInfo.magCal.tempSlope[0], 
        tcmInfo.magCal.tempSlope[1], 
        tcmInfo.magCal.tempSlope[2]);
}

void tcmPlot(int serial_plot) {
	// For use with serial plotting tools like SerialPlot
    switch (serial_plot) {
    case 0:
		printf("Serial plotting disabled (serial_plot=0).\n");
        break;
    case 1:
        printf("%d %0.2f %0.2f %0.2f\n",
            serial_plot,
            tcmInfo.headingDeg,
            tcmInfo.current.north,
            tcmInfo.current.east);
        break;
    case 2:
        printf("%d %0.2f %0.2f %0.2f\n",
            serial_plot,
            tcmInfo.orientation.rollRad,
            tcmInfo.orientation.pitchRad,
            tcmInfo.orientation.yawRad);
        break;
    case 3:
        printf("%d %d %d %d %0.2f %0.2f %0.2f\n",
            serial_plot,
            tcmInfo.raw.acc.x,
            tcmInfo.raw.acc.y,
            tcmInfo.raw.acc.z,
            tcmInfo.scaled.acc.x,
            tcmInfo.scaled.acc.y,
            tcmInfo.scaled.acc.z);
        break;
    case 4:
        printf("%d %d %d %d %0.2f %0.2f %0.2f\n",
            serial_plot,
            tcmInfo.raw.mag.x,
            tcmInfo.raw.mag.y,
            tcmInfo.raw.mag.z,
            tcmInfo.scaled.mag.x,
            tcmInfo.scaled.mag.y,
            tcmInfo.scaled.mag.z);
        break;
    case 5:
        printf("%d %d %0.2f %d %0.2f\n",
            serial_plot,
            tcmInfo.raw.temp, tcmInfo.scaled.temp, tcmInfo.raw.batt, tcmInfo.scaled.batt);
        break;
    default:
        printf("Need to set 'serial_plot' to enable plotting.\n");
        break;
    }
}

// ------------------------------
// Initialization and Run
// ------------------------------
bool initTcm(int log_level) {
    esp_log_level_set(TAG, log_level);

    // Wait to allow TCM to power up
    ESP_LOGI(TAG, "Delaying %d ms for TCM startup...", STARTUP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
    ESP_LOGI(TAG, "Continuing initialization...");

    resetAverages();
	defaultCalibrations();
	defaultRaws();
	
    // Connect to TCM
    int loopCnt = 0;
        while (!connectDeviceUsb(TCM_PID, TCM_VID)) {
            if (++loopCnt * TCM_CONNECT_DELAY_MS >= TCM_CONNECT_TIMEOUT_MS) {
                ESP_LOGE(TAG, "TCM failed to USB connect.");
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(TCM_CONNECT_DELAY_MS));
        }


	// Get TCM Info
    // Version and Serial Number do not need error checking
    getStrUsb(tcmInfo.version, sizeof(tcmInfo.version), FIRMWARE_VERSION_CMD);
    ESP_LOGI(TAG, "TCM Version: %s", tcmInfo.version);
    getStrUsb(tcmInfo.serialNum, sizeof(tcmInfo.serialNum), SERIAL_NUMBER_CMD);
	ESP_LOGI(TAG, "TCM Serial Number: %s", tcmInfo.serialNum);
    ESP_LOGI(TAG, "Waiting 5 seconds before reading calibrations...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    float junk; // Placeholder for unused values
    if (!getFloatAscii85Usb(&junk, "RVN13", CALIBRATION_CMD, "06000008")) {
        ESP_LOGE(TAG, "Failed to get RVN13");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMO, "TMO", CALIBRATION_CMD, "06080008")) {
        ESP_LOGE(TAG, "Failed to get TMO");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMR, "TMR", CALIBRATION_CMD, "06100008")) {
        ESP_LOGE(TAG, "Failed to get TMR");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMA, "TMA", CALIBRATION_CMD, "06180008")) {
        ESP_LOGE(TAG, "Failed to get TMA");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMB, "TMB", CALIBRATION_CMD, "06200008")) {
        ESP_LOGE(TAG, "Failed to get TMB");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMC, "TMC", CALIBRATION_CMD, "06280008")) {
        ESP_LOGE(TAG, "Failed to get TMC");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[0][0], "AXX", CALIBRATION_CMD, "06300008")) {
        ESP_LOGE(TAG, "Failed to get AXX");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[0][1], "AXY", CALIBRATION_CMD, "06380008")) {
        ESP_LOGE(TAG, "Failed to get AXY");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[0][2], "AXZ", CALIBRATION_CMD, "06400008")) {
        ESP_LOGE(TAG, "Failed to get AXZ");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[1][0], "AYX", CALIBRATION_CMD, "06480008")) {
        ESP_LOGE(TAG, "Failed to get AYX");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[1][1], "AYY", CALIBRATION_CMD, "06500008")) {
        ESP_LOGE(TAG, "Failed to get AYY");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[1][2], "AYZ", CALIBRATION_CMD, "06580008")) {
        ESP_LOGE(TAG, "Failed to get AYZ");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[2][0], "AZX", CALIBRATION_CMD, "06600008")) {
        ESP_LOGE(TAG, "Failed to get AZX");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[2][1], "AZY", CALIBRATION_CMD, "06680008")) {
        ESP_LOGE(TAG, "Failed to get AZY");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[2][2], "AZZ", CALIBRATION_CMD, "06700008")) {
        ESP_LOGE(TAG, "Failed to get AZZ");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.offset[0], "AXV", CALIBRATION_CMD, "06780008")) {
        ESP_LOGE(TAG, "Failed to get AXV");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.offset[1], "AYV", CALIBRATION_CMD, "06800008")) {
        ESP_LOGE(TAG, "Failed to get AYV");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.offset[2], "AZV", CALIBRATION_CMD, "06880008")) {
        ESP_LOGE(TAG, "Failed to get AZV");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.cubic[0], "AXC", CALIBRATION_CMD, "06900008")) {
        ESP_LOGE(TAG, "Failed to get AXC");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.cubic[1], "AYC", CALIBRATION_CMD, "06980008")) {
        ESP_LOGE(TAG, "Failed to get AYC");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.cubic[2], "AZC", CALIBRATION_CMD, "06A00008")) {
        ESP_LOGE(TAG, "Failed to get AZC");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[0][0], "MXX", CALIBRATION_CMD, "06A80008")) {
        ESP_LOGE(TAG, "Failed to get MXX");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[0][1], "MXY", CALIBRATION_CMD, "06B00008")) {
        ESP_LOGE(TAG, "Failed to get MXY");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[0][2], "MXZ", CALIBRATION_CMD, "06B80008")) {
        ESP_LOGE(TAG, "Failed to get MXZ");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[1][0], "MYX", CALIBRATION_CMD, "06C00008")) {
        ESP_LOGE(TAG, "Failed to get MYX");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[1][1], "MYY", CALIBRATION_CMD, "06C80008")) {
        ESP_LOGE(TAG, "Failed to get MYY");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[1][2], "MYZ", CALIBRATION_CMD, "06D00008")) {
        ESP_LOGE(TAG, "Failed to get MYZ");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[2][0], "MZX", CALIBRATION_CMD, "06D80008")) {
        ESP_LOGE(TAG, "Failed to get MZX");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[2][1], "MZY", CALIBRATION_CMD, "06E00008")) {
        ESP_LOGE(TAG, "Failed to get MZY");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[2][2], "MZZ", CALIBRATION_CMD, "06E80008")) {
        ESP_LOGE(TAG, "Failed to get MZZ");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.hardIron[0], "MXV", CALIBRATION_CMD, "06F00008")) {
        ESP_LOGE(TAG, "Failed to get MXV");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.hardIron[1], "MYV", CALIBRATION_CMD, "06F80008")) {
        ESP_LOGE(TAG, "Failed to get MYV");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.hardIron[2], "MZV", CALIBRATION_CMD, "06000108")) {
        ESP_LOGE(TAG, "Failed to get MZV");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempRef, "MRF", CALIBRATION_CMD, "06080108")) {
        ESP_LOGE(TAG, "Failed to get MRF");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempSlope[0], "TMX", CALIBRATION_CMD, "06100108")) {
        ESP_LOGE(TAG, "Failed to get TMX");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempSlope[1], "TMY", CALIBRATION_CMD, "06180108")) {
        ESP_LOGE(TAG, "Failed to get TMY");
        return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempSlope[2], "TMZ", CALIBRATION_CMD, "06200108")) {
        ESP_LOGE(TAG, "Failed to get TMZ");
        return false;
    }

    dispCalibrations();

    ESP_LOGI(TAG, "TCM Initialized");
    return true;
}

bool runTcm(int serial_plot)
{
    //Note: The only calculation that can be in error is temperature.
    //      This will be handled by using a defaulut value 
    bool success = getSensorsRawUsb(&tcmInfo.raw, SENSOR_READINGS_CMD);
    if (!success) {
        ESP_LOGE(TAG, "Failed to get raw sensor data");
        return false;
    }
    addRawsToRawSum();
    if (tcmAvg.sampleCount == NUM_ITERATIONS_TO_AVERAGE) {
        ESP_LOGI(TAG, "Averaged %d samples", NUM_ITERATIONS_TO_AVERAGE);
        calcAveragesAndCopyToRaw();
        resetAverages();
        calcTcm();
        dispTcm();
        tcmPlot(serial_plot);
    }
    else {
        calcTcm();
        dispTcm();
        tcmPlot(serial_plot);
    }
	return true;
}
