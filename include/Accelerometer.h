//H3LIS331DL defines
//TODO: all of these definitions need to be configured? idk but these
// look like pins and registers on the device driver of the accelerometer
// which need reconfiguration if different.

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H
#include <cstdint>
#include <Wire.h>
#include "Arduino.h"

#define ADDR_ACCEL 0x19 

#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define HP_FILTER_RESET 0x25
#define REFERENCE 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28 
#define OUT_X_H 0x29 
#define OUT_Y_L 0x2A 
#define OUT_Y_H 0x2B 
#define OUT_Z_L 0x2C 
#define OUT_Z_H 0x2D 
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_THS 0x32
#define INT1_DURATION 0x33
#define INT2_CFG 0x34
#define INT2_SRC 0x35
#define INT2_THS 0x36
#define INT2_DURATION 0x37
#define NORMAL_MODE_50HZ_ALL_AXES 0x27


uint8_t writeI2CReg8Blocking(uint8_t addr, uint8_t subaddr, uint8_t data);
void readI2CRegNBlocking(uint8_t addr, uint8_t subaddr, uint8_t buflen, uint8_t *buf);
void configAccelerometer();

//we read from the accelerometer much slower than the accelerometer's data rate to make sure we always get new data
//reading the same data twice could mess with the prediction algorithms
//a better method is to use the interrupt pin on the accelerometer that tells us every time new data is available

static unsigned long measurementPeriod;//in microseconds. Represents 100Hz

static uint16_t robotPeriod[2];//measured in microseconds per degree, with some memory for discrete integration

//this is the times we measured the accelerometer at. We keep some history for extrapolation
static unsigned long accelMeasTime[2];

//this angle (degrees) is calculated only using the accelerometer. We keep it separate to keep our discrete integration algorithms operating smoothly
//the beacon sets our heading to 0, which would mess up the discrete integration if allowed to affect this variable directly
//instead we utilize a trim variable. In Accel control mode, the user controls trim with the encoder wheel. in hybrid mode, the beacon controls trim
static uint16_t accelAngle;

//in degrees, this angle is added to the accel angle as adjusted by the beacon or the driver
static  uint16_t accelTrim;

static uint16_t angleAtLastMeasurement;

//we make this global so that comms can send it out for calibration purposes
static int16_t zAccel;

//TODO: implement flip. For now I think the controls will just be inverted?
static byte flip;


void runAccelerometer();

#endif // ACCELEROMETER_H