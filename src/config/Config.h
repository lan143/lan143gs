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

#ifndef H_CONFIG_H
#define H_CONFIG_H

#include <EEPROM.h>

#define GET_CONFIG Config::getInstance()->getConfig()

typedef struct AccCal_s {
    bool calibrated = false;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
} AccCal_t;

typedef struct CompassCal_s {
    bool calibrated = false;
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
} CompassCal_t;

typedef struct WiFi_s {
    bool isAPMode = true;
    char apSSID[64];
    char apPassword[64];
    char clientSSID[64];
    char clientPassword[64];
} WiFi_t;

typedef struct Config_s {
    AccCal_t accZero;
    WiFi_t wifi;
    CompassCal_t compassZero;
} Config_t;

class Config {
public:
    static Config *getInstance() {
        if (!_instance) {
            _instance = new Config();
        }

        return _instance;
    }

    void init();
    void save();

    Config_t* getConfig() { return _config; }

protected:
    EEPROMClass *_eeprom;
    Config_t *_config;

private:
    static Config *_instance;

    Config() {}
    Config(const Config &);
    Config &operator=(Config &);
};

#endif
