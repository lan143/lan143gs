/**
 * MIT License
 *
 * Copyright (c) 2021 Kravchenko Artyom
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Arduino.h"
#include "AccGyroDriver.h"
#include "../common/vector.h"
#include "../common/calibration.h"
#include "../config/Config.h"

void AccGyroDriver::init() {
    driverInit();
    uint16_t accLpfCutHz = 10;
    uint32_t accSampleTimeUs = 1e6 / 100;

    for (int axis = 0; axis < 3; axis++) {
        _accFilter[axis].init(accLpfCutHz, accSampleTimeUs, BIQUAD_Q, FILTER_LPF);
    }

    if (GET_CONFIG->accZero.calibrated) {
        accZero[X] = GET_CONFIG->accZero.x;
        accZero[Y] = GET_CONFIG->accZero.y;
        accZero[Z] = GET_CONFIG->accZero.z;
        accCalibrationState = ZERO_CALIBRATION_DONE;
    }

    zeroCalibrationStartV(&gyroCalibration, CALIBRATING_GYRO_TIME_MS, 32, false);
}

imuData_t AccGyroDriver::getData() {
    updateData();
    updateGyroData();
    updateAccData();

    return data;
}

void AccGyroDriver::updateAccData() {
    for (int axis = 0; axis < 3; axis++) {
        accADCRaw[axis] = _accFilter[axis].applyFilter(accADCRaw[axis]);
    }

    if (accCalibrationState != ZERO_CALIBRATION_DONE) {
        performAccCalibration();
    }

    for (int axis = 0; axis < 3; axis++) {
        accADCRaw[axis] -= accZero[axis];
    }

    applySensorAlignment(accADCRaw, accADCRaw, GET_CONFIG->accZero.alignment);

    data.accX = (float)accADCRaw[X] / accScale;
    data.accY = (float)accADCRaw[Y] / accScale;
    data.accZ = (float)accADCRaw[Z] / accScale;

    // Before filtering check for clipping and vibration levels
    if (fabsf(data.accX) > ACC_CLIPPING_THRESHOLD_G || fabsf(data.accY) > ACC_CLIPPING_THRESHOLD_G || fabsf(data.accZ) > ACC_CLIPPING_THRESHOLD_G) {
        accIsClipped = true;
        accClipCount++;
    } else {
        accIsClipped = false;
    }
}

void AccGyroDriver::startAccCalibration() {
    GET_CONFIG->accZero.x = 0.0f;
    GET_CONFIG->accZero.y = 0.0f;
    GET_CONFIG->accZero.z = 0.0f;
    GET_CONFIG->accZero.calibrated = false;
    Config::getInstance()->save();

    accCalibrationState = ZERO_CALIBRATION_NONE;
}

void AccGyroDriver::performAccCalibration() {
    for (int axis = 0; axis < 3; axis++) {
        // Reset a[axis] at start of calibration
        if (accCalibrationState == ZERO_CALIBRATION_NONE) {
            _a[axis] = 0;
            accCalibrationState = ZERO_CALIBRATION_IN_PROGRESS;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        _a[axis] += accADCRaw[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        accADCRaw[axis] = 0;
    }

    if (accCalibrationCycles == 0) {
        accZero[X] = (_a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accZero[Y] = (_a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accZero[Z] = (_a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - accScale;

        GET_CONFIG->accZero.x = accZero[X];
        GET_CONFIG->accZero.y = accZero[Y];
        GET_CONFIG->accZero.z = accZero[Z];
        GET_CONFIG->accZero.calibrated = true;
        Config::getInstance()->save();

        accCalibrationState = ZERO_CALIBRATION_DONE;
    }

    accCalibrationCycles--;
}

void AccGyroDriver::updateGyroData() {
    if (zeroCalibrationIsCompleteV(&gyroCalibration)) {
        int32_t gyroADCtmp[XYZ_AXIS_COUNT];

        // Copy gyro value into int32_t (to prevent overflow) and then apply calibration and alignment
        gyroADCtmp[X] = (int32_t)gyroADCRaw[X] - (int32_t)gyroZero[X];
        gyroADCtmp[Y] = (int32_t)gyroADCRaw[Y] - (int32_t)gyroZero[Y];
        gyroADCtmp[Z] = (int32_t)gyroADCRaw[Z] - (int32_t)gyroZero[Z];

        applySensorAlignment(gyroADCtmp, gyroADCtmp, GET_CONFIG->accZero.alignment);

        data.gyroX = (float)gyroADCtmp[X] * gyroScale;
        data.gyroY = (float)gyroADCtmp[Y] * gyroScale;
        data.gyroZ = (float)gyroADCtmp[Z] * gyroScale;
    } else {
        performGyroCalibration();

        data.gyroX = 0.0f;
        data.gyroY = 0.0f;
        data.gyroZ = 0.0f;
    }
}

void AccGyroDriver::performGyroCalibration() {
    fpVector3_t v;

    v.v[0] = gyroADCRaw[0];
    v.v[1] = gyroADCRaw[1];
    v.v[2] = gyroADCRaw[2];

    zeroCalibrationAddValueV(&gyroCalibration, &v);

    // Check if calibration is complete after this cycle
    if (zeroCalibrationIsCompleteV(&gyroCalibration)) {
        zeroCalibrationGetZeroV(&gyroCalibration, &v);
        gyroZero[0] = v.v[0];
        gyroZero[1] = v.v[1];
        gyroZero[2] = v.v[2];
    } else {
        gyroZero[0] = 0;
        gyroZero[1] = 0;
        gyroZero[2] = 0;
    }
}
