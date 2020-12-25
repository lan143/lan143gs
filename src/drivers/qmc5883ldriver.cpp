
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
#include "QMC5883LDriver.h"

QMC5883LDriver::QMC5883LDriver() {
    _compass = new QMC5883L();
}

void QMC5883LDriver::driverInit() {
    _compass->init();

    Serial.print("QMC5883L: ");
    Serial.println(_compass->ready() ? "OK" : "FAIL");

    _config = new compassConfig_t();
    _config->mag_declination = 0;
}

void QMC5883LDriver::update() {
    int16_t t;
    int16_t data[XYZ_AXIS_COUNT];
    _compass->readRaw(&data[X], &data[Y], &data[Z], &t);

    for (uint8_t axis = X; axis < XYZ_AXIS_COUNT; axis++) {
        _magADCRaw[axis] = data[axis];
    }
}