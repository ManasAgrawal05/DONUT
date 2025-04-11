#include "Arduino.h"
#include <Wire.h>
#include "Accelerometer.h"

// Adjust this to 0x18 or 0x19 depending on your hardware
#define H3LIS331DL_I2C_ADDR  0x19

// Register addresses (check the H3LIS331DL datasheet for details)
#define WHO_AM_I       0x0F
#define CTRL_REG2      0x21
#define CTRL_REG3      0x22
#define STATUS_REG     0x27
#define OUT_X_L        0x28
// X_H=0x29, Y_L=0x2A, Y_H=0x2B, Z_L=0x2C, Z_H=0x2D

// Example config values (these are typical, but verify in the datasheet)
#define NORMAL_MODE_50HZ_ALL_AXES 0x27
//   bit layout for CTRL_REG1 = 0b00100111 
//   - DR=00 => 50 Hz
//   - PM=10 => Normal mode
//   - X/Y/Z enable bits = 1

void readXYZ(int16_t &x, int16_t &y, int16_t &z);

void setup() {
  Serial.begin(115200);
  Wire.begin();  // uses default SDA=21, SCL=22 on ESP32
  


  // Configure accelerometer: Normal mode, 50 Hz, X/Y/Z enabled
  configAccelerometer();

}

void loop() {
  // Read 6 bytes for X, Y, Z
  int16_t x, y, z;
  readXYZ(x, y, z);

  Serial.print("X = ");
  Serial.print(x);
  Serial.print(", Y = ");
  Serial.print(y);
  Serial.print(", Z = ");
  Serial.println(z);

  // Convert raw data to g’s if needed
  // float sensitivity = <depends on chosen range>; 
  // float x_g = x * sensitivity;
  // float y_g = y * sensitivity;
  // float z_g = z * sensitivity;

  delay(250);
}

// Helper function: Read X, Y, Z raw data (6 consecutive registers)
void readXYZ(int16_t &x, int16_t &y, int16_t &z) {
  // On some LSM/LIS devices, you can set MSB of sub-address to enable auto-increment
  // For H3LIS331DL, auto-increment is typically bit 7 of the register address
  // So we can do something like (OUT_X_L | 0x80).
  // Check your datasheet to confirm or if you need single reads.
  
  Wire.beginTransmission(H3LIS331DL_I2C_ADDR);
  Wire.write(OUT_X_L | 0x80); // 0x80 for auto-increment
  Wire.endTransmission(false);

  // Request 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  Wire.requestFrom(H3LIS331DL_I2C_ADDR, (uint8_t)6);

  // Read the 6 bytes
  if (Wire.available() == 6) {
    uint8_t xL = Wire.read();
    uint8_t xH = Wire.read();
    uint8_t yL = Wire.read();
    uint8_t yH = Wire.read();
    uint8_t zL = Wire.read();
    uint8_t zH = Wire.read();
    
    x = (int16_t)((xH << 8) | xL);
    y = (int16_t)((yH << 8) | yL);
    z = (int16_t)((zH << 8) | zL);
  }
}

