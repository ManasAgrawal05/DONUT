#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.h"

void setup() {
  // put your setup code here, to run once:
  Wire.begin(21, 22);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  runAcceleromter();
  Serial.println("zAccel: " + zAccel);
}