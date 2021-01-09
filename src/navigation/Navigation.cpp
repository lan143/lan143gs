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
#include "Navigation.h"
#include "../factories/GNSSDriverFactory.h"
#include "../pid/PID.h"

Navigation* Navigation::_instance = 0;

Navigation::Navigation() {
    _imu = new IMU();
    _gnss = GNSSDriverFactory::build();
    _servoAngle = new TrackerServo(SERVO_ANGLE);
}

void Navigation::init() {
    _imu->init();
    _gnss->init();
}

void Navigation::aimingUpdate(unsigned long currentTime) {
    float dT = currentTime - _previousAimingUpdateTime;
    _previousAimingUpdateTime = currentTime;

    // 1. Get current Euler angles
    attitudeEulerAngles_t attitudeData = _imu->getAttitudeData(currentTime);
    Serial.print("Roll: ");
    Serial.print(attitudeData.values.roll);
    Serial.print("\t");

    Serial.print("Pitch: ");
    Serial.print(attitudeData.values.pitch);
    Serial.print("\t");

    Serial.print("Yaw: ");
    Serial.print(attitudeData.values.yaw);
    Serial.println();
    // 2. Calculate setpoint for yaw and pitch axis
    int setpoint = 0;
    Serial.print("setpoint: ");
    Serial.print(setpoint);
    Serial.println();
    // 3. Run PID regulator for yaw and pitch axis
    int influence = computePID(abs(attitudeData.values.pitch), setpoint, 0.1f, 0.0000005f, 0.005f, dT);

    if (setpoint < attitudeData.values.pitch) {
        influence *= -1;
    }

    Serial.print("influence: ");
    Serial.print(influence);
    Serial.println();
    // 4. Execute PID sum in servos
    _servoAngle->update(influence);
}

void Navigation::coordsUpdate() {
    _gnssData = _gnss->getData();

    Serial.print("GPS Ok: ");
    Serial.print(_gnssData.ok ? "true" : "false");
    Serial.print("\t");

    Serial.print("Sats: ");
    Serial.print(_gnssData.sats);
    Serial.print("\t");
    
    Serial.print("Lat: ");
    Serial.print(_gnssData.lat);
    Serial.print("\t");
    
    Serial.print("Lng: ");
    Serial.print(_gnssData.lng);
    Serial.print("\t");
    
    Serial.print("Altitude: ");
    Serial.print(_gnssData.height);
    Serial.println();
}

void Navigation::update() {
    if ((millis() - this->_lastUpdateAimingTime >= AIMING_LOOP_TIME) || this->_lastUpdateAimingTime == 0) {
        this->aimingUpdate(millis());
        this->_lastUpdateAimingTime = millis();
    }

    if ((millis() - this->_lastUpdateGNSSTime >= GNNS_LOOP_TIME) || this->_lastUpdateGNSSTime == 0) {
        //this->coordsUpdate();
        this->_lastUpdateGNSSTime = millis();
    }
}
