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

#include "UBloxDriver.h"
#include "mapping.h"

UBloxDriver::UBloxDriver() {
    _serial = &Serial2;
    _serial->begin(9800);
    _gps = new SFE_UBLOX_GPS();
}

void UBloxDriver::init() {
    if (_gps->begin(*_serial)) {
        Serial.println("UBloxDriver inited");
    } else {
        Serial.println("Failed to init UBloxDriver");
    }
}

gnssData_t UBloxDriver::getData() {
    gnssData_t data;

    if (_gps->isConnected()) {
        data.ok = true;
        data.lat = (float)_gps->getLatitude() / pow(10, 7);
        data.lng = (float)_gps->getLongitude() / pow(10, 7);
        data.height = _gps->getAltitudeMSL() / 10;
        data.sats = _gps->getSIV();
    } else {
        data.ok = false;
    }

    return data;
}
