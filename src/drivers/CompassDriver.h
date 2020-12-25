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

#ifndef H_COMPASS_H
#define H_COMPASS_H

#include "Arduino.h"
#include "../common/maths.h"
#include "../common/calibration.h"
#include "../common/axis.h"

#define COMPASS_CALIBRATION_TIME 30000

typedef struct compassData_s {
    int16_t x;
    int16_t y;
    int16_t z;
} compassData_t;

typedef struct compassConfig_s {
    int16_t mag_declination;                // Get your magnetic declination from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
} compassConfig_t;

class CompassDriver {
public:
    void init();
    virtual void driverInit();
    compassConfig_t* getConfig()
    {
        return _config;
    }
    virtual bool isReady();
    bool isCalibated() { return _calibrationState == ZERO_CALIBRATION_DONE; }

    void startCalibration();
    zeroCalibrationState_e getCalibrationState() { return _calibrationState; }

    compassData_t getData();

protected:
    virtual void update();

    void performCalibration();

protected:
    compassConfig_t* _config;

    zeroCalibrationState_e _calibrationState;
    int32_t _zeros[XYZ_AXIS_COUNT];
    timeMs_t _calStartedAt;
    int32_t _magPrev[XYZ_AXIS_COUNT];
    sensorCalibrationState_t _calState;

    int32_t _magADCRaw[XYZ_AXIS_COUNT];
};

#endif
