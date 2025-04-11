// #include <Arduino.h>
// #include <Adafruit_DotStar.h>
// #include <SPI.h>
// #include <i2c_t3.h>
// // #include "src"
// //#include <Wire.h>

// #define enablePin 22 //Manas: my understanding is HIGH = off, Low = on
// #define PIN_IR 15


// unsigned long lastReceived = 0;

// //controls
// byte flip = 0;
// int16_t thumbX = 0;
// int16_t thumbY = 0;
// uint16_t throt = 0;
// uint16_t head = 0;
// byte en = 0; //stands for enable, I think AA means on, 0 means off
// //leds
// //TODO: idk what Adafruit_DotStar is or what the params are.
// Adafruit_DotStar strip = Adafruit_DotStar(5, DOTSTAR_GBR);

// //**********************//
// // MELTYBRAIN VARIABLES //
// //**********************//

// int16_t angle = 0;//LSB is one degree. Our current heading

// int16_t meltyAngle = 0;//the commanded bearing angle for meltybrain control
// uint16_t meltyThrottle = 0;

// uint8_t senseMode = ACCEL_SENSING;

// static unsigned const int HIGHEST_RPM = 0;

// //ACCELEROMETER
// void configAccelerometer(void);

// //states
// uint8_t state = 1;

// #define STATE_IDLE 1
// #define STATE_SPIN 2

// void runMeltyBrain(void);




// void goIdle() {
//   state = STATE_IDLE;
//   digitalWrite(enablePin, HIGH);
//   setMotorSpeed(motor1, 0);
//   setMotorSpeed(motor2, 0);
  
// }


// void goSpin() {
//   state = STATE_SPIN;
//   digitalWrite(enablePin, LOW);
// }

// void feedWatchdog() {
//   noInterrupts();
//   WDOG_REFRESH = 0xA602;
//   WDOG_REFRESH = 0xB480;
//   interrupts();
// }

// //this runs if the robot code hangs! cut off the motors
// void watchdog_isr() {
//   digitalWrite(enablePin, HIGH);
// }

// void setup() {
//   Serial.begin(57600);
//   // SPI.begin();
//   // pinMode(enablePin, OUTPUT);
//   // digitalWrite(enablePin, HIGH);
  

//   // pinMode(PIN_IR, INPUT);

//   // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 1800000, I2C_OP_MODE_IMM);//1.8MHz clock rate


//   // //TODO: wtf is watchdog? From by understand 
//   // //SETUP WATCHDOG
//   // //settings taken from: https://bigdanzblog.wordpress.com/2017/10/27/watch-dog-timer-wdt-for-teensy-3-1-and-3-2/
//   // noInterrupts();
//   // WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
//   // WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
//   // delayMicroseconds(1);

//   // WDOG_TOVALH = 0x006d; //1 second timer
//   // WDOG_TOVALL = 0xdd00;
//   // WDOG_PRESC = 0x400;
//   // WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | 
//   //                 WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN |
//   //                 WDOG_STCTRLH_CLKSRC | WDOG_STCTRLH_IRQRSTEN;
//   // interrupts();

//   // NVIC_ENABLE_IRQ(IRQ_WDOG);//enable watchdog interrupt

//   // analogWriteFrequency(3, 250);//this changes the frequency of both motor outputs

//   configAccelerometer();


//   // configComms();

//   // goIdle();
// }

// void loop() {

//   // //Bark bark
//   // feedWatchdog();

//   // //check for incoming messages
//   // loopComms();

//   // //make sure comms haven't timed out
//   // if(micros() - lastReceived > 1000*1000 && state != STATE_IDLE) {
//   //   en = 0x0;
//   //   goIdle();
//   // }

//   runAccelerometer();

//   // switch(state) {
//   //   case STATE_IDLE:
//   //     break;
//   //   case STATE_SPIN:
//   //     runMeltyBrain();//manage all of the sensors and predict our current heading

//   //     if(en != 0xAA) {
//   //       goIdle();
//   //     }
//   //   default:
//   //     break;
//   // }
// }



