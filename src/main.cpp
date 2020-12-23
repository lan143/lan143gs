#include <Arduino.h>
#include "Wire.h"
#include "mapping.h"
#include "navigation/Navigation.h"
#include "config/Config.h"
#include "wifi/WiFiMgr.h"
#include "web/WebServer.h"

Navigation navigation;
WebServer webServer;

void initI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
}

void initSerial() {
  Serial.begin(115200);

  while (!Serial) {}
}

void setup() {
  initSerial();
  Config::getInstance()->init();
  WiFiMgr::getInstance()->init();
  webServer.init();
  initI2C();
  navigation.init();
}

void loop() {
  navigation.update();
}
