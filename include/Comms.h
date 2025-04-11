#include "Arduino.h"
#include "Accelerometer.h"

#define CH1 33
#define CH2 35
#define CH3 32

static int thumbX = 0;
static int thumbY = 0;
static int throt = 0;

static int16_t angle = 0;//LSB is one degree. Our current heading

static int16_t meltyAngle = 0;//the commanded bearing angle for meltybrain control
static uint16_t meltyThrottle = 0;
static unsigned long lastReceived = 0;

void configComms();
void loopComms();
