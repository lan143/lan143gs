/**
 * MIT License
 *
 * Copyright (c) 2020 Kravchenko Artyom
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

#ifndef H_ACC_GYRO_DRIVER_H
#define H_ACC_GYRO_DRIVER_H

#include "../common/axis.h"
#include "../common/calibration.h"
#include "../filters/BiquadFilter.h"

#define CALIBRATING_GYRO_TIME_MS            2000
#define ACC_CLIPPING_THRESHOLD_G            7.9f
#define CALIBRATING_ACC_CYCLES              400

enum AccCalibrationState {
    STATE_NOT_STARTED = 0,
    STATE_IN_PROGRESS = 1,
    STATE_COMPLETE = 2,
};

typedef struct imuData_s {
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
} imuData_t;

class AccGyroDriver {
public:
    void init();

    imuData_t getData();

    void startAccCalibration();
    AccCalibrationState getAccCalibrationState() { return accCalibrationState; }
protected:
    virtual void driverInit();
    virtual void updateData();

    void updateGyroData();
    void updateAccData();

    void performGyroCalibration();
    void performAccCalibration();

protected:
    float gyroScale;
    int32_t accScale;
    bool accIsClipped;
    int32_t accClipCount;

    imuData_t data;

    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    int16_t accADCRaw[XYZ_AXIS_COUNT];

    BiquadFilter _accFilter[XYZ_AXIS_COUNT];

    AccCalibrationState accCalibrationState = STATE_NOT_STARTED;
    uint32_t accCalibrationCycles = CALIBRATING_ACC_CYCLES;
    int64_t _a[XYZ_AXIS_COUNT];

    zeroCalibrationVector_t gyroCalibration;

    int16_t gyroZero[XYZ_AXIS_COUNT];
    int16_t accZero[XYZ_AXIS_COUNT];
};

#endif
