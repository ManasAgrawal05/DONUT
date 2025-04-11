
#include <Accelerometer.h>

//write a single byte to an I2C device
uint8_t writeI2CReg8Blocking(uint8_t addr, uint8_t subaddr, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(subaddr);
  Wire.write(data);
  return Wire.endTransmission();
}

//read N bytes from an I2C device
uint8_t readI2CRegNBlocking(uint8_t addr, uint8_t subaddr, uint8_t buflen, uint8_t *buf) {
  Wire.beginTransmission(addr);
  Wire.write(subaddr | 0x80);//the current accelerometer requires that the msb be set high to do a multi-byte transfer
  uint8_t error = Wire.endTransmission();
  Wire.requestFrom(addr, buflen);
  // while(Wire.available()) *(buf++) = Wire.readBytes();
  while (Wire.available())
  {
    /* code */
    Wire.readBytes(buf, sizeof(uint8_t));
  }
  
  return error;
}

void configAccelerometer() {
  //TODO: make sure we're writing the correct bytes here.
  writeI2CReg8Blocking(ADDR_ACCEL, CTRL_REG1, 0x2E);//1000Hz, normal mode, YZ enabled
  writeI2CReg8Blocking(ADDR_ACCEL, CTRL_REG4, 0x30);//block data update enabled, 400g full scale
}



void runAccelerometer() {
  //if we're past the interval to measure, get a new measurement.
  if(micros() - accelMeasTime[0] > measurementPeriod) {
    //shift all of the old values down
    for(int i=1; i>0; i--) {
      accelMeasTime[i] = accelMeasTime[i-1];
    }
    //put in the new value
    accelMeasTime[0] = micros();
    
    uint8_t accelBuf[6];

    //this uses blocking I2C, which makes it relatively slow. But given that we run our I2C ar 1.8MHz we will likely be okay
    readI2CRegNBlocking(ADDR_ACCEL, OUT_X_L, 6, accelBuf);

    int16_t xAccel = (((int16_t) accelBuf[1]) << 8) | (int16_t) accelBuf[0];//represents acceleration tangential to the ring, not useful to us
    int16_t yAccel = (((int16_t) accelBuf[3]) << 8) | (int16_t) accelBuf[2];//represents acceleration axial to the ring, which shows which way the bot is flipped
    zAccel = (((int16_t) accelBuf[5]) << 8) | (int16_t) accelBuf[4];//represents acceleration radial to the ring, which is a measure of rotation speed
    Serial.println("xAccel: " + xAccel);
    Serial.println( "yAccel: " + yAccel);
    Serial.println("\nzAccel: " + zAccel);
    
    //shift all of the old values down
    for(int i=1; i>0; i--) {
      robotPeriod[i] = robotPeriod[i-1];
    }
    
    //put in the new value
    //this equation has been carefully calibrated for this bot. See here for explanation:
    //https://www.swallenhardware.io/battlebots/2018/8/12/halo-pt-9-accelerometer-calibration
    robotPeriod[0] = (uint32_t) (726 / sqrt((double) (zAccel-225)/522));

    //give up if the bot is moving too slowly
    if(zAccel < 400) return;

    //find the new angle
    //TRIANGULAR INTEGRATION
    uint32_t deltaT = accelMeasTime[0] - accelMeasTime[1];
    angleAtLastMeasurement = (angleAtLastMeasurement + (deltaT/robotPeriod[0] + deltaT/robotPeriod[1])/2) % 360;

    accelAngle = angleAtLastMeasurement;
    
  } 
  //if it isn't time to check the accelerometer, predict our current heading
  else {
    //predict the current velocity by extrapolating old data
    uint32_t newTime = micros();
    uint32_t periodPredicted = robotPeriod[1] + (newTime - accelMeasTime[1]) * (robotPeriod[0] - robotPeriod[1]) / (accelMeasTime[0] - accelMeasTime[1]);
    //predict the current robot heading by triangular integration up to the extrapolated point
    uint32_t deltaT = newTime - accelMeasTime[0];
    accelAngle = (angleAtLastMeasurement + (deltaT/periodPredicted + deltaT/robotPeriod[0])/2) % 360;
  }
}

