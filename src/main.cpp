#include <Arduino.h>
#include "Wire.h"
#include "mapping.h"
#include "navigation/NavigationService.h"

NavigationService navigation;

void initI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
}

void initSerial() {
  Serial.begin(115200);

  while (!Serial) {}
}

void setup() {
  initSerial();
  initI2C();
  navigation.init();
}

void loop() {
  navigation.update();
}
