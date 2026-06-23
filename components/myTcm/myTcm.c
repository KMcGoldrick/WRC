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

static rawSensors _prevRaw = { {0,0,0}, {0,0,0}, 0, 0 };
static uint8_t _stuckCounters[3] = { 0 }; // mag.x,y,z
static bool _prevRawInitialized = false;

// Global TCM info
TcmInfo tcmInfo;
TcmAverage tcmAvg;

// Mode items
bool tcmDebug = false;
int dataSelect = 0;
bool tcmDataAsText = false;
bool tcmAverage = false;

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
    tcmInfo.magCal.tempRef = 0.0f;
    tcmInfo.magCal.tempSlope[0] = 0.0f;
    tcmInfo.magCal.tempSlope[1] = 0.0f;
    tcmInfo.magCal.tempSlope[2] = 0.0f;
    if (tcmDebug) ESP_LOGI(TAG, "Set Default Calibrations");
    return true;
}

bool defaultRaws(void) {
    tcmInfo.raw.batt = 3700;      // mV
    tcmInfo.raw.temp = 2500;      // Raw temperature ADC count
    tcmInfo.raw.acc = (rawXYZ){ 512, 0, 512 };
    tcmInfo.raw.mag = (rawXYZ){ 100, 200, -50 };
    ESP_LOGI(TAG, "Set Default Raw Sensor Values");
    return true;
}

// ------------------------------
// Temperature and Battery
// ------------------------------
float calcBattV(void) {
    return tcmInfo.raw.batt / 1000.0f;
}
float calcTempC(void)
{
    // Ensure calibration TMR is non-zero
    if (tcmInfo.tempCal.TMR == 0.0f) {
        ESP_LOGW(TAG, "Calibration error: TMR is zero");
        return DEFAULT_TEMP;
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

    float tempK = 1.0f / denom;
    float tempC = tempK - 273.15f;

    return tempC;
}
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

    return (XYZ) { calibrated[0], calibrated[1], calibrated[2] };
}
XYZ calcTempCompMag(void) {
    float raw[3] = { tcmInfo.raw.mag.x, tcmInfo.raw.mag.y, tcmInfo.raw.mag.z };
    float corrected[3], calibrated[3], tempComp[3];

    // Use calibrated temperature in degC, not the raw ADC count
    float temp = tcmInfo.scaled.temp;  // already computed by calcTempC() in calcTcm()

    // Clamp to calibration range in degrees C
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

    return (XYZ) { tempComp[0], tempComp[1], tempComp[2] };
}
RPY calcRPY(void) {
    RPY rpy;

    // Roll: atan2(acc.y, acc.z)
    rpy.rollRad = atan2f(tcmInfo.scaled.acc.y, tcmInfo.scaled.acc.z);

    // Pitch: roll-projected denominator matches Lowell Instruments reference implementation.
    // ay*sin(roll) + az*cos(roll) is the geometrically correct gravity projection
    // onto the roll-corrected forward axis.
    rpy.pitchRad = atan2f(
        -tcmInfo.scaled.acc.x,
        tcmInfo.scaled.acc.y * sinf(rpy.rollRad) +
        tcmInfo.scaled.acc.z * cosf(rpy.rollRad)
    );

    // Tilt-compensated magnetometer projection onto horizontal plane
    float by = tcmInfo.scaled.mag.z * sinf(rpy.rollRad)
        - tcmInfo.scaled.mag.y * cosf(rpy.rollRad);
    float bx = tcmInfo.scaled.mag.x * cosf(rpy.pitchRad)
        + tcmInfo.scaled.mag.y * sinf(rpy.pitchRad) * sinf(rpy.rollRad)
        + tcmInfo.scaled.mag.z * sinf(rpy.pitchRad) * cosf(rpy.rollRad);

    // Yaw: atan2(by, bx)
    rpy.yawRad = atan2f(by, bx);

    return rpy;
}
float calcTilt(void) {
    float ax = tcmInfo.scaled.acc.x;
    float ay = tcmInfo.scaled.acc.y;
    float az = tcmInfo.scaled.acc.z;

    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag == 0.0f) return 0.0f;

    float tilt = acosf(az / mag);

    // Fold upside-down deployments back into [0, 90°]
    if (tilt > M_PI / 2.0f) tilt = M_PI - tilt;

    return tilt;  // radians, [0, pi/2]
}
float speedFromTilt(void) {
    float tiltDeg = tcmInfo.tiltRad * 180.0f / M_PI;

    // Clamp to table range
    if (tiltDeg <= _tiltCurve[0].tiltDeg)
        return _tiltCurve[0].speed;
    if (tiltDeg >= _tiltCurve[_tiltCurveLen - 1].tiltDeg)
        return _tiltCurve[_tiltCurveLen - 1].speed;

    // Find bracketing points and interpolate
    for (int i = 0; i < _tiltCurveLen - 1; i++) {
        if (tiltDeg >= _tiltCurve[i].tiltDeg && tiltDeg < _tiltCurve[i + 1].tiltDeg) {
            float t = (tiltDeg - _tiltCurve[i].tiltDeg) /
                (_tiltCurve[i + 1].tiltDeg - _tiltCurve[i].tiltDeg);
            return _tiltCurve[i].speed + t * (_tiltCurve[i + 1].speed - _tiltCurve[i].speed);
        }
    }

    return 0.0f; // unreachable if table is well-formed
}
float calcHeading(void) {
    float heading = tcmInfo.orientation.yawRad * 180.0f / M_PI;
    heading = fmodf(heading + 180.0f + DECLINATION_DEG, 360.0f);
    if (heading < 0.0f) heading += 360.0f;
    // Removed erroneous "- 180.0f" that was here previously.
    // heading is now a proper [0, 360) compass bearing.
    return heading;
}
Velocity calcCurrent(void) {
    Velocity vel;
    vel.north = tcmInfo.speed * cosf(tcmInfo.headingDeg * M_PI / 180.0f);
    vel.east = tcmInfo.speed * sinf(tcmInfo.headingDeg * M_PI / 180.0f);
    return vel;
}

void calcTcm(void) {
    tcmInfo.scaled.batt = calcBattV();
    tcmInfo.scaled.temp = calcTempC();       // must precede calcTempCompMag
    tcmInfo.scaled.acc = calcAcc();
    tcmInfo.scaled.mag = calcTempCompMag(); // uses tcmInfo.scaled.temp
    tcmInfo.orientation = calcRPY();
	tcmInfo.tiltRad = calcTilt();
    tcmInfo.speed = speedFromTilt();
    tcmInfo.headingDeg = calcHeading();
    tcmInfo.current = calcCurrent();
}

// ------------------------------
// Averaging
// ------------------------------
void addRawsToRawSum(void) {
    tcmAvg.sampleCount++;
    if (tcmDebug) ESP_LOGI(TAG, "Adding sample %d of %d", tcmAvg.sampleCount, NUM_ITERATIONS_TO_AVERAGE);

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
    if (tcmDebug) ESP_LOGI(TAG, "Averages reset");
}

void _update_prev_raw(const rawSensors* cur)
{
    _prevRaw = *cur;
    for (int i = 0; i < 3; ++i) _stuckCounters[i] = 0;
    _prevRawInitialized = true;
}

/*
 * checkRawValuesNotStuck
 * - returns true when raw sensor fields are not persistently stuck at zero
 * - returns false if any raw channel has been zero for RAW_STUCK_THRESHOLD consecutive checks
 */
bool checkRawValuesNotStuck(void)
{
    const rawSensors cur = tcmInfo.raw;

    if (!_prevRawInitialized) {
        _update_prev_raw(&cur);
        return true;
    }

    int32_t curVals[3] = {
        cur.mag.x, cur.mag.y, cur.mag.z
    };
    int32_t prevVals[3] = {
        _prevRaw.mag.x, _prevRaw.mag.y, _prevRaw.mag.z
    };

    bool anyStuck = false;

    for (int i = 0; i < 3; ++i) {
        if (curVals[i] == 0) {
            if (curVals[i] == prevVals[i]) {
                if (_stuckCounters[i] < 255) _stuckCounters[i]++;
            }
            else {
                _stuckCounters[i] = 1;
            }
        }
        else {
            _stuckCounters[i] = 0;
        }

        if (_stuckCounters[i] >= RAW_STUCK_THRESHOLD) {
            anyStuck = true;
        }
    }

    _prevRaw = cur;

    if (anyStuck) {
        ESP_LOGE(TAG, "Detected mag channel stuck at 0 for >= %d checks", RAW_STUCK_THRESHOLD);
        const char* names[3] = { "mag.x", "mag.y", "mag.z" };
        for (int i = 0; i < 3; ++i) {
            if (_stuckCounters[i] >= RAW_STUCK_THRESHOLD) {
                ESP_LOGE(TAG, "  Stuck channel: %s (value=0, count=%d)", names[i], _stuckCounters[i]);
            }
        }
        return false;
    }

    return true;
}
// ------------------------------
// TCM Display
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
    ESP_LOGI(TAG, "Tilt (deg): %.2f  Speed: %.3f m/s", tcmInfo.tiltRad * 180.0f / M_PI, tcmInfo.speed);
    ESP_LOGI(TAG, "Heading (deg): %.2f  [0=N, 90=E, 180=S, 270=W]", tcmInfo.headingDeg);
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

void tcmDataBinary(int select)
{
#define TCM_BIN_SOF             0xAA
    uint8_t frame[51];
    uint8_t idx = 0;

    frame[idx++] = TCM_BIN_SOF;
    frame[idx++] = select;

    uint8_t* lenPtr = &frame[idx++];
    uint8_t payloadStart = idx;

    switch (select)
    {
    case 0:
        break;

    case 1:
        memcpy(&frame[idx], &tcmInfo.headingDeg, sizeof(float)); idx += 4;
        memcpy(&frame[idx], &tcmInfo.current.north, sizeof(float)); idx += 4;
        memcpy(&frame[idx], &tcmInfo.current.east, sizeof(float)); idx += 4;
        break;

    case 2:
        memcpy(&frame[idx], &tcmInfo.orientation.rollRad, sizeof(float)); idx += 4;
        memcpy(&frame[idx], &tcmInfo.orientation.pitchRad, sizeof(float)); idx += 4;
        memcpy(&frame[idx], &tcmInfo.orientation.yawRad, sizeof(float)); idx += 4;
        break;

    case 3:
        memcpy(&frame[idx], &tcmInfo.raw.acc, sizeof(rawXYZ)); idx += sizeof(rawXYZ);
        memcpy(&frame[idx], &tcmInfo.scaled.acc, sizeof(XYZ)); idx += sizeof(XYZ);
        break;

    case 4:
        memcpy(&frame[idx], &tcmInfo.raw.mag, sizeof(rawXYZ)); idx += sizeof(rawXYZ);
        memcpy(&frame[idx], &tcmInfo.scaled.mag, sizeof(XYZ)); idx += sizeof(XYZ);
        break;

    case 5:
        memcpy(&frame[idx], &tcmInfo.raw.temp, sizeof(uint16_t)); idx += 2;
        memcpy(&frame[idx], &tcmInfo.scaled.temp, sizeof(float)); idx += 4;
        memcpy(&frame[idx], &tcmInfo.raw.batt, sizeof(uint16_t)); idx += 2;
        memcpy(&frame[idx], &tcmInfo.scaled.batt, sizeof(float)); idx += 4;
        break;

    case 6:
        memcpy(&frame[idx], tcmInfo.version, sizeof(tcmInfo.version)); idx += sizeof(tcmInfo.version);
        memcpy(&frame[idx], tcmInfo.serialNum, sizeof(tcmInfo.serialNum)); idx += sizeof(tcmInfo.serialNum);
        break;

    case 7:
        memcpy(&frame[idx], &tcmInfo.tempCal, sizeof(TempCalCoef));
        idx += sizeof(TempCalCoef);
        break;

    case 8:
        memcpy(&frame[idx], tcmInfo.accCal.offset, 3 * sizeof(float)); idx += 12;
        memcpy(&frame[idx], tcmInfo.accCal.cubic, 3 * sizeof(float)); idx += 12;
        break;

    case 9:
        memcpy(&frame[idx], tcmInfo.accCal.gain, 9 * sizeof(float));
        idx += 36;
        break;

    case 10:
        memcpy(&frame[idx], tcmInfo.magCal.softIron, 9 * sizeof(float)); idx += 36;
        memcpy(&frame[idx], tcmInfo.magCal.hardIron, 3 * sizeof(float)); idx += 12;
        break;

    case 11:
        memcpy(&frame[idx], &tcmInfo.magCal.tempRef, sizeof(float)); idx += 4;
        memcpy(&frame[idx], tcmInfo.magCal.tempSlope, 3 * sizeof(float)); idx += 12;
        break;

    case 12:
        memcpy(&frame[idx], &tcmInfo.tiltRad, sizeof(float)); idx += 4;
        memcpy(&frame[idx], &tcmInfo.speed, sizeof(float)); idx += 4;
        break;

    default:
        break;
    }

    *lenPtr = idx - payloadStart;

    send_rs485_bytes((const uint8_t*)frame, idx);
}

void tcmDataText(int select)
{
    char uartMessage[192];

    switch (select)
    {
    case 0:
        snprintf(uartMessage, sizeof(uartMessage), "0,0,0,0\n");
        break;

    case 1:
        snprintf(uartMessage, sizeof(uartMessage),
            "1,%.3f,%.3f,%.3f\n",
            tcmInfo.headingDeg,
            tcmInfo.current.north,
            tcmInfo.current.east);
        break;

    case 2:
        snprintf(uartMessage, sizeof(uartMessage),
            "2,%.3f,%.3f,%.3f\n",
            tcmInfo.orientation.rollRad,
            tcmInfo.orientation.pitchRad,
            tcmInfo.orientation.yawRad);
        break;

    case 3:
        snprintf(uartMessage, sizeof(uartMessage),
            "3,%d,%d,%d,%.3f,%.3f,%.3f\n",
            tcmInfo.raw.acc.x,
            tcmInfo.raw.acc.y,
            tcmInfo.raw.acc.z,
            tcmInfo.scaled.acc.x,
            tcmInfo.scaled.acc.y,
            tcmInfo.scaled.acc.z);
        break;

    case 4:
        snprintf(uartMessage, sizeof(uartMessage),
            "4,%d,%d,%d,%.3f,%.3f,%.3f\n",
            tcmInfo.raw.mag.x,
            tcmInfo.raw.mag.y,
            tcmInfo.raw.mag.z,
            tcmInfo.scaled.mag.x,
            tcmInfo.scaled.mag.y,
            tcmInfo.scaled.mag.z);
        break;

    case 5:
        snprintf(uartMessage, sizeof(uartMessage),
            "5,%u,%.3f,%u,%.3f\n",
            (unsigned)tcmInfo.raw.temp,
            tcmInfo.scaled.temp,
            (unsigned)tcmInfo.raw.batt,
            tcmInfo.scaled.batt);
        break;

    case 6:
        snprintf(uartMessage, sizeof(uartMessage),
            "6,%s,%s\n",
            tcmInfo.version,
            tcmInfo.serialNum);
        break;

    case 7:
        snprintf(uartMessage, sizeof(uartMessage),
            "7,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            tcmInfo.tempCal.TMO,
            tcmInfo.tempCal.TMR,
            tcmInfo.tempCal.TMA,
            tcmInfo.tempCal.TMB,
            tcmInfo.tempCal.TMC);
        break;

    case 8:
        snprintf(uartMessage, sizeof(uartMessage),
            "8,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            tcmInfo.accCal.offset[0],
            tcmInfo.accCal.offset[1],
            tcmInfo.accCal.offset[2],
            tcmInfo.accCal.cubic[0],
            tcmInfo.accCal.cubic[1],
            tcmInfo.accCal.cubic[2]);
        break;

    case 9:
        snprintf(uartMessage, sizeof(uartMessage),
            "9,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            tcmInfo.accCal.gain[0][0],
            tcmInfo.accCal.gain[0][1],
            tcmInfo.accCal.gain[0][2],
            tcmInfo.accCal.gain[1][0],
            tcmInfo.accCal.gain[1][1],
            tcmInfo.accCal.gain[1][2],
            tcmInfo.accCal.gain[2][0],
            tcmInfo.accCal.gain[2][1],
            tcmInfo.accCal.gain[2][2]);
        break;

    case 10:
        snprintf(uartMessage, sizeof(uartMessage),
            "10,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            tcmInfo.magCal.softIron[0][0],
            tcmInfo.magCal.softIron[0][1],
            tcmInfo.magCal.softIron[0][2],
            tcmInfo.magCal.softIron[1][0],
            tcmInfo.magCal.softIron[1][1],
            tcmInfo.magCal.softIron[1][2],
            tcmInfo.magCal.softIron[2][0],
            tcmInfo.magCal.softIron[2][1],
            tcmInfo.magCal.softIron[2][2],
            tcmInfo.magCal.hardIron[0],
            tcmInfo.magCal.hardIron[1],
            tcmInfo.magCal.hardIron[2]);
        break;

    case 11:
        snprintf(uartMessage, sizeof(uartMessage),
            "11,%.5f,%.5f,%.5f,%.5f\n",
            tcmInfo.magCal.tempRef,
            tcmInfo.magCal.tempSlope[0],
            tcmInfo.magCal.tempSlope[1],
            tcmInfo.magCal.tempSlope[2]);
        break;

    case 12:
        snprintf(uartMessage, sizeof(uartMessage),
            "12,%.5f,%.3f\n",
            tcmInfo.tiltRad * 180.0f / M_PI,   // tilt in degrees
            tcmInfo.speed);
        break;
    
    default:
        snprintf(uartMessage, sizeof(uartMessage), "-1,0,0,0\n");
        break;
    }

    send_rs485_text(uartMessage);
}

// ------------------------------
// Initialization and Run
// ------------------------------
bool initTcm(int level, bool debug, int select, bool asText) {

    esp_log_level_set(TAG, level);
    tcmDebug = debug;
    dataSelect = select;
    tcmDataAsText = asText;

    if (tcmDebug) ESP_LOGI(TAG, "Delaying %d ms for TCM startup...", STARTUP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
    if (tcmDebug) ESP_LOGI(TAG, "Continuing initialization...");

    resetAverages();
    defaultCalibrations();
    defaultRaws();

    int loopCnt = 0;
    while (!connectDeviceUsb(TCM_PID, TCM_VID)) {
        if (++loopCnt * TCM_CONNECT_DELAY_MS >= TCM_CONNECT_TIMEOUT_MS) {
            ESP_LOGE(TAG, "TCM failed to USB connect.");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(TCM_CONNECT_DELAY_MS));
    }

    getStrUsb(tcmInfo.version, sizeof(tcmInfo.version), FIRMWARE_VERSION_CMD);
    if (tcmDebug) ESP_LOGI(TAG, "TCM Version: %s", tcmInfo.version);
    getStrUsb(tcmInfo.serialNum, sizeof(tcmInfo.serialNum), SERIAL_NUMBER_CMD);
    if (tcmDebug) ESP_LOGI(TAG, "TCM Serial Number: %s", tcmInfo.serialNum);
    if (tcmDebug) ESP_LOGI(TAG, "Waiting 5 seconds before reading calibrations...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    float junk;
    if (!getFloatAscii85Usb(&junk, "RVN13", CALIBRATION_CMD, "06000008")) {
        ESP_LOGE(TAG, "Failed to get RVN13"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMO, "TMO", CALIBRATION_CMD, "06080008")) {
        ESP_LOGE(TAG, "Failed to get TMO"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMR, "TMR", CALIBRATION_CMD, "06100008")) {
        ESP_LOGE(TAG, "Failed to get TMR"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMA, "TMA", CALIBRATION_CMD, "06180008")) {
        ESP_LOGE(TAG, "Failed to get TMA"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMB, "TMB", CALIBRATION_CMD, "06200008")) {
        ESP_LOGE(TAG, "Failed to get TMB"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.tempCal.TMC, "TMC", CALIBRATION_CMD, "06280008")) {
        ESP_LOGE(TAG, "Failed to get TMC"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[0][0], "AXX", CALIBRATION_CMD, "06300008")) {
        ESP_LOGE(TAG, "Failed to get AXX"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[0][1], "AXY", CALIBRATION_CMD, "06380008")) {
        ESP_LOGE(TAG, "Failed to get AXY"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[0][2], "AXZ", CALIBRATION_CMD, "06400008")) {
        ESP_LOGE(TAG, "Failed to get AXZ"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[1][0], "AYX", CALIBRATION_CMD, "06480008")) {
        ESP_LOGE(TAG, "Failed to get AYX"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[1][1], "AYY", CALIBRATION_CMD, "06500008")) {
        ESP_LOGE(TAG, "Failed to get AYY"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[1][2], "AYZ", CALIBRATION_CMD, "06580008")) {
        ESP_LOGE(TAG, "Failed to get AYZ"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[2][0], "AZX", CALIBRATION_CMD, "06600008")) {
        ESP_LOGE(TAG, "Failed to get AZX"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[2][1], "AZY", CALIBRATION_CMD, "06680008")) {
        ESP_LOGE(TAG, "Failed to get AZY"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.gain[2][2], "AZZ", CALIBRATION_CMD, "06700008")) {
        ESP_LOGE(TAG, "Failed to get AZZ"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.offset[0], "AXV", CALIBRATION_CMD, "06780008")) {
        ESP_LOGE(TAG, "Failed to get AXV"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.offset[1], "AYV", CALIBRATION_CMD, "06800008")) {
        ESP_LOGE(TAG, "Failed to get AYV"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.offset[2], "AZV", CALIBRATION_CMD, "06880008")) {
        ESP_LOGE(TAG, "Failed to get AZV"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.cubic[0], "AXC", CALIBRATION_CMD, "06900008")) {
        ESP_LOGE(TAG, "Failed to get AXC"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.cubic[1], "AYC", CALIBRATION_CMD, "06980008")) {
        ESP_LOGE(TAG, "Failed to get AYC"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.accCal.cubic[2], "AZC", CALIBRATION_CMD, "06A00008")) {
        ESP_LOGE(TAG, "Failed to get AZC"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[0][0], "MXX", CALIBRATION_CMD, "06A80008")) {
        ESP_LOGE(TAG, "Failed to get MXX"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[0][1], "MXY", CALIBRATION_CMD, "06B00008")) {
        ESP_LOGE(TAG, "Failed to get MXY"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[0][2], "MXZ", CALIBRATION_CMD, "06B80008")) {
        ESP_LOGE(TAG, "Failed to get MXZ"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[1][0], "MYX", CALIBRATION_CMD, "06C00008")) {
        ESP_LOGE(TAG, "Failed to get MYX"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[1][1], "MYY", CALIBRATION_CMD, "06C80008")) {
        ESP_LOGE(TAG, "Failed to get MYY"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[1][2], "MYZ", CALIBRATION_CMD, "06D00008")) {
        ESP_LOGE(TAG, "Failed to get MYZ"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[2][0], "MZX", CALIBRATION_CMD, "06D80008")) {
        ESP_LOGE(TAG, "Failed to get MZX"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[2][1], "MZY", CALIBRATION_CMD, "06E00008")) {
        ESP_LOGE(TAG, "Failed to get MZY"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.softIron[2][2], "MZZ", CALIBRATION_CMD, "06E80008")) {
        ESP_LOGE(TAG, "Failed to get MZZ"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.hardIron[0], "MXV", CALIBRATION_CMD, "06F00008")) {
        ESP_LOGE(TAG, "Failed to get MXV"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.hardIron[1], "MYV", CALIBRATION_CMD, "06F80008")) {
        ESP_LOGE(TAG, "Failed to get MYV"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.hardIron[2], "MZV", CALIBRATION_CMD, "06000108")) {
        ESP_LOGE(TAG, "Failed to get MZV"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempRef, "MRF", CALIBRATION_CMD, "06080108")) {
        ESP_LOGE(TAG, "Failed to get MRF"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempSlope[0], "TMX", CALIBRATION_CMD, "06100108")) {
        ESP_LOGE(TAG, "Failed to get TMX"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempSlope[1], "TMY", CALIBRATION_CMD, "06180108")) {
        ESP_LOGE(TAG, "Failed to get TMY"); return false;
    }
    if (!getFloatAscii85Usb(&tcmInfo.magCal.tempSlope[2], "TMZ", CALIBRATION_CMD, "06200108")) {
        ESP_LOGE(TAG, "Failed to get TMZ"); return false;
    }

    dispCalibrations();

    if (tcmDebug) ESP_LOGI(TAG, "TCM Initialized");
    return true;
}

void outputData(int error)
{
    if (tcmDebug)
    {
        dispTcm();
    }
    else
    {
        if (error == OK) {
            if (tcmDataAsText) {
                tcmDataText(dataSelect);
            }
            else {
                tcmDataBinary(dataSelect);
            }
        }
        else if (error == NORaw) {
            if (tcmDataAsText) {
                send_rs485_text("ERROR: No raw sensor data\n");
            }
            else {
                uint8_t errFrame[3] = { 0xAA, 0xFF, 0x00 };
                send_rs485_bytes(errFrame, sizeof(errFrame));
            }
        }
        else if (error == RAWStuck) {
            if (tcmDataAsText) {
                send_rs485_text("ERROR: Raw sensor values stuck at zero\n");
            }
            else {
                uint8_t errFrame[3] = { 0xAA, 0xFE, 0x00 };
                send_rs485_bytes(errFrame, sizeof(errFrame));
            }
        }
    }
}

bool runTcm(bool debug, bool average, bool dataAsText, int select)
{
    tcmDebug = debug;
    tcmAverage = average;
    tcmDataAsText = dataAsText;
    dataSelect = select;

    bool success = getSensorsRawUsb(&tcmInfo.raw, SENSOR_READINGS_CMD);
    if (!success) {
        send_commandUsb(RESET_CMD);
        ESP_LOGE(TAG, "Failed to get raw sensor data");
        outputData(NORaw);
        return false;
    }

    // FIX #4: checkRawValuesNotStuck() re-enabled.
    // Was previously commented out with "success = true" which silently
    // disabled the stuck-sensor safety check. If you need to bypass this
    // check during development, define SKIP_STUCK_CHECK in WRCDefs.h and
    // gate it there rather than commenting out the call here.
    success = checkRawValuesNotStuck();
    if (!success) {
        send_commandUsb(RESET_CMD);
        ESP_LOGE(TAG, "Aborting TCM calculation due to stuck raw sensor values");
        outputData(RAWStuck);
        return false;
    }

    if (tcmAverage)
    {
        addRawsToRawSum();
        if (tcmAvg.sampleCount == NUM_ITERATIONS_TO_AVERAGE) {
            if (tcmDebug) ESP_LOGI(TAG, "Averaged %d samples", NUM_ITERATIONS_TO_AVERAGE);
            calcAveragesAndCopyToRaw();
            resetAverages();
            calcTcm();
            outputData(OK);
        }
    }
    else
    {
        calcTcm();
        outputData(OK);
    }
    return true;
}
