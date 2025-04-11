// //Channel 1 is left/right
// //channel 2 is up/down
// //channel 3 is throttle
// //channel 4 


#include "Comms.h"


int x_dir = 0;
int y_dir = 0;
int throttle = 0;

void configComms() {
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
}

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}


void loopComms() {
  thumbX = readChannel(CH1, -100, 100, 0);
  thumbY = readChannel(CH2, -100, 100, 0);
  throt = readChannel(CH3, -100, 100, -100);

  meltyThrottle = sqrt(thumbX*thumbX + thumbY*thumbY)/2;
  int16_t calcAngle = (int16_t) (atan2((double) thumbY, (double) thumbX*(flip*2-1))*180.0/PI);
  if(calcAngle < 0) calcAngle += 360;
  meltyAngle = (uint16_t) calcAngle;
  
  /*
   * transmitter failsafe is setting all values to 0, so as long as we
   * have a nonzero response, we know we're transmitting.
   */
  if (thumbX != 0 || thumbY != 0 || throt != 0) {
    lastReceived = micros();
  }

}

// void receivePacket() {
//   //TODO: how do we know we're flipped? are we just gonna have inverted controls?
//   flip = (stat & 0x02) > 0;//indicates whether the bot is inverted. 1 is inverted, 0 is normal

//   //TODO: how do we calculate heading?
  
//   head = ((uint16_t) packet[7]) << 8 | ((uint16_t) packet[8]);
//   //TODO: how do we calculate enable?
//   en = packet[9];
  

//   /*/
  
//   /*/this code is used in calibration testing
//   packet[1] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0xFF000000) >> 24);
//   packet[2] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x00FF0000) >> 16);
//   packet[3] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x0000FF00) >> 8);
//   packet[4] = (byte) ((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x000000FF);
//   packet[5] = (byte) ((zAccel & 0xFF00) >> 8);
//   packet[6] = (byte) (zAccel & 0x00FF);

//   Serial1.write(packet, 7);
//   //*/
// }
