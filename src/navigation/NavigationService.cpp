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

#include "Arduino.h"
#include "NavigationService.h"
#include "../factory/ImuFactory.h"
#include "../factory/CompassFactory.h"
#include "../factory/GNSSFactory.h"

NavigationService::NavigationService() {
    _imu = ImuFactory::build();
    _compass = CompassFactory::build();
    _gnss = GNSSFactory::build();
}

void NavigationService::init() {
    _imu->init();
    _compass->init();
    _gnss->init();
}

void NavigationService::aimingUpdate() {
    imuData_t imuData = _imu->getData();
    compassData_t compassData = _compass->getData();
}

void NavigationService::coordsUpdate() {
    _gnssData = _gnss->getData();
}

void NavigationService::update() {
    if ((millis() - this->_lastUpdateAimingTime >= AIMING_LOOP_TIME) || this->_lastUpdateAimingTime == 0) {
        this->aimingUpdate();
        this->_lastUpdateAimingTime = millis();
    }

    if ((millis() - this->_lastUpdateGNSSTime >= GNNS_LOOP_TIME) || this->_lastUpdateGNSSTime == 0) {
        this->coordsUpdate();
        this->_lastUpdateGNSSTime = millis();
    }
}
