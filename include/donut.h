
#include <Wire.h>
#include "Arduino.h"
#include <SPI.h>
// #include <i2c_t3.h>

#include "meltybrain.h"


#define STATE_IDLE 1
#define STATE_SPIN 2

static uint8_t state = 1;
