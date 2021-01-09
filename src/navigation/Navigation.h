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

#ifndef H_NAVIGATION_SERVISE_H
#define H_NAVIGATION_SERVISE_H

#include <Arduino.h>
#include "IMU.h"
#include "../drivers/GNSSDriver.h"
#include "../mapping.h"
#include "../servos/TrackerServo.h"

#define AIMING_LOOP_TIME 10 // 100 Hz
#define GNNS_LOOP_TIME 1000 // 1 Hz

class Navigation {
public:
    static Navigation *getInstance() {
        if (!_instance) {
            _instance = new Navigation();
        }

        return _instance;
    }

    void init();
    void update();

    void startAccCalibration() { _imu->startAccCalibration(); }
    void startCompassCalibration() { _imu->startCompassCalibration(); }
    zeroCalibrationState_e getAccCalibrationState() { return _imu->getAccCalibrationState(); }
    zeroCalibrationState_e getCompassCalibrationState() { return _imu->getCompassCalibration(); }

protected:

    void aimingUpdate(unsigned long currentTime);
    void coordsUpdate();

protected:
    IMU* _imu;
    GNSSDriver* _gnss;

    gnssData_s _gnssData;

    TrackerServo* _servoAngle;

    unsigned long _lastUpdateAimingTime = 0;
    unsigned long _lastUpdateGNSSTime = 0;
    unsigned long _previousAimingUpdateTime = 0;
private:
    static Navigation *_instance;

    Navigation();
    Navigation(const Navigation &);
    Navigation &operator=(Navigation &);
};

#endif