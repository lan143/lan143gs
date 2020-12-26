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

#include "CompassDriver.h"
#include "../config/Config.h"
#include "../common/maths.h"

void CompassDriver::init() {
    driverInit();

    if (GET_CONFIG->compassZero.calibrated) {
        _zeros[X] = GET_CONFIG->compassZero.x;
        _zeros[Y] = GET_CONFIG->compassZero.y;
        _zeros[Z] = GET_CONFIG->compassZero.z;
        _calibrationState = ZERO_CALIBRATION_DONE;
    }
}

void CompassDriver::startCalibration() {
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
        _zeros[axis] = 0;
    }

    _zeros[X] = 0;
    _zeros[Y] = 0;
    _zeros[Z] = 0;
    Config::getInstance()->save();

    sensorCalibrationResetState(&_calState);
    _calibrationState = ZERO_CALIBRATION_IN_PROGRESS;
    _calStartedAt = millis();
}

compassData_t CompassDriver::getData() {
    performCalibration();

    for (uint8_t axis = X; axis < XYZ_AXIS_COUNT; axis++) {
        _magADCRaw[axis] -= _zeros[axis];
    }

    compassData_t data;
    data.x = _magADCRaw[X];
    data.y = _magADCRaw[Y];
    data.z = _magADCRaw[Z];

    return data;
}

void CompassDriver::performCalibration() {
    if (_calibrationState == ZERO_CALIBRATION_IN_PROGRESS) {
        if (millis() - _calStartedAt < COMPASS_CALIBRATION_TIME) {
            float diffMag = 0;
            float avgMag = 0;

            for (int axis = 0; axis < 3; axis++) {
                diffMag += (_magADCRaw[axis] - _magPrev[axis]) * (_magADCRaw[axis] - _magPrev[axis]);
                avgMag += (_magADCRaw[axis] + _magPrev[axis]) * (_magADCRaw[axis] + _magPrev[axis]) / 4.0f;
            }

            // sqrtf(diffMag / avgMag) is a rough approximation of tangent of angle between magADC and magPrev. tan(8 deg) = 0.14
            if ((avgMag > 0.01f) && ((diffMag / avgMag) > (0.14f * 0.14f))) {
                sensorCalibrationPushSampleForOffsetCalculation(&_calState, _magADCRaw);

                for (int axis = 0; axis < 3; axis++) {
                    _magPrev[axis] = _magADCRaw[axis];
                }
            }
        } else {
            float magZerof[3];
            sensorCalibrationSolveForOffset(&_calState, magZerof);

            for (int axis = 0; axis < 3; axis++) {
                _zeros[axis] = lrintf(magZerof[axis]);
            }

            GET_CONFIG->compassZero.calibrated = true;
            GET_CONFIG->compassZero.x = _zeros[X];
            GET_CONFIG->compassZero.y = _zeros[Y];
            GET_CONFIG->compassZero.z = _zeros[Z];
            Config::getInstance()->save();

            _calibrationState = ZERO_CALIBRATION_DONE;
        }
    }
}