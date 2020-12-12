#include <Arduino.h>
#include "Wire.h"
#include "mapping.h"
#include "navigation/NavigationService.h"
#include <SoftwareSerial.h>

NavigationService navigation;
SoftwareSerial gnssSerial(GNSS_RX, GNSS_TX);

void initI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
}

void initSerial() {
  Serial.begin(115200);

  while (!Serial) {}

  gnssSerial.begin(4800);
  gnssSerial.println("Hello, world?");
}

void setup() {
  initSerial();
  initI2C();
  navigation.init();
}

void loop() {
  navigation.update();
}
