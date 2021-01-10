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

#include <AsyncJson.h>
#include <ArduinoJson.h>
#include "WebServer.h"
#include "../navigation/Navigation.h"

WebServer::WebServer() {
    _server = new AsyncWebServer(80);
}

void WebServer::init() {
    _server->begin();
    _server->on("/api/", HTTP_GET, [this](AsyncWebServerRequest *request) { version(request); });
    _server->on("/api/calibrate/acc", HTTP_GET, [this](AsyncWebServerRequest *request) { calibrateAccStatus(request); });
    _server->on("/api/calibrate/acc", HTTP_POST, [this](AsyncWebServerRequest *request) { startCalibrateAcc(request); });
    _server->on("/api/calibrate/compass", HTTP_GET, [this](AsyncWebServerRequest *request) { calibrateCompassStatus(request); });
    _server->on("/api/calibrate/compass", HTTP_POST, [this](AsyncWebServerRequest *request) { startCalibrateCompass(request); });
}

void WebServer::version(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument jd(50);
    jd["version"] = "0.0.0";
    serializeJson(jd, *response);
    request->send(response);
}

void WebServer::startCalibrateAcc(AsyncWebServerRequest *request) {
    Navigation::getInstance()->startAccCalibration();

    AsyncWebServerResponse *response = request->beginResponse(201, "application/json");
    request->send(response);
}

void WebServer::calibrateAccStatus(AsyncWebServerRequest *request) {
    auto state = Navigation::getInstance()->getAccCalibrationState();

    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument jd(50);
    jd["state"] = state;
    serializeJson(jd, *response);
    request->send(response);
}

void WebServer::startCalibrateCompass(AsyncWebServerRequest *request) {
    Navigation::getInstance()->startCompassCalibration();

    AsyncWebServerResponse *response = request->beginResponse(201, "application/json");
    request->send(response);
}

void WebServer::calibrateCompassStatus(AsyncWebServerRequest *request) {
    auto state = Navigation::getInstance()->getCompassCalibrationState();

    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument jd(50);
    jd["state"] = state;
    serializeJson(jd, *response);
    request->send(response);
}
