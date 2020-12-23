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

#include "WiFiMgr.h"
#include "../config/Config.h"

WiFiMgr* WiFiMgr::_instance = 0;

void WiFiMgr::init() {
    if (GET_CONFIG->wifi.isAPMode) {
        if (!_wifi->softAP(GET_CONFIG->wifi.apSSID, GET_CONFIG->wifi.apPassword)) {
            strncpy(GET_CONFIG->wifi.apSSID, "lan143gs", sizeof(GET_CONFIG->wifi.apSSID));
            strncpy(GET_CONFIG->wifi.apPassword, "1234567890", sizeof(GET_CONFIG->wifi.apPassword));
            Config::getInstance()->save();
            init();
        }
    } else {
        _wifi->begin(GET_CONFIG->wifi.clientSSID, GET_CONFIG->wifi.clientPassword);
    }
}
