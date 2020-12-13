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

#ifndef H_NAVIGATION_SERVISE_H
#define H_NAVIGATION_SERVISE_H

#include "IMU.h"
#include "../drivers/GNSSDriver.h"
#include "../mapping.h"

#define AIMING_LOOP_TIME 10 // 100 Hz
#define GNNS_LOOP_TIME 100 // 10 Hz

class NavigationService {
public:
    NavigationService();
    void init();

    void update();
protected:

    void aimingUpdate(unsigned long currentTime);
    void coordsUpdate();

protected:
    IMU* _imu;
    GNSSDriver* _gnss;

    gnssData_s _gnssData;

    unsigned long _lastUpdateAimingTime = 0;
    unsigned long _lastUpdateGNSSTime = 0;
};

#endif