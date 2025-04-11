#include "Arduino.h"
#include <esp32-hal.h>
#include "Accelerometer.h"
#include "Comms.h"


static int motor1 = 3;
static int motor2 = 4;

void setMotorSpeed(int motor, int spd);
void runMeltyBrain();