#include <Arduino.h>
#include "Wire.h"
#include "mapping.h"
#include "navigation/Navigation.h"
#include "config/Config.h"
#include "wifi/WiFiMgr.h"
#include "web/WebServer.h"

WebServer webServer;

void initI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
}

void initSerial() {
  Serial.begin(115200);

  while (!Serial) {}
}

void initServices() {
  Config::getInstance()->init();
  WiFiMgr::getInstance()->init();
  webServer.init();
  Navigation::getInstance()->init();
}

void setup() {
  initSerial();
  initI2C();
  initServices();
}

void loop() {
  Navigation::getInstance()->update();
}
