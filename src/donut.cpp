#include <donut.h>



// #define enablePin 22 //Manas: my understanding is HIGH = off, Low = on
// #define PIN_IR 15




// //controls controlled by comms

// //leds
// //TODO: idk what Adafruit_DotStar is or what the params are.
// Adafruit_DotStar strip = Adafruit_DotStar(5, DOTSTAR_GBR);

// //**********************//
// // MELTYBRAIN VARIABLES //
// //**********************//



// uint8_t senseMode = ACCEL_SENSING;

// static unsigned const int HIGHEST_RPM = 0;

// //states






void goIdle() {
  state = STATE_IDLE;
//   digitalWrite(enablePin, HIGH);
  setMotorSpeed(3, 0);
  setMotorSpeed(4, 0);
  
}


void goSpin() {
  state = STATE_SPIN;
//   digitalWrite(enablePin, LOW);
}

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

void setup() {
  Serial.begin(115200);
//   // SPI.begin();
//   // pinMode(enablePin, OUTPUT);
//   // digitalWrite(enablePin, HIGH);
  

//   // pinMode(PIN_IR, INPUT);

  Wire.begin();


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

  configAccelerometer();


//   // configComms();

//   // goIdle();
}

void loop() {

//   // //Bark bark
//   // feedWatchdog();

  //check for incoming messages
  loopComms();

//   // //make sure comms haven't timed out
//   // if(micros() - lastReceived > 1000*1000 && state != STATE_IDLE) {
//   //   en = 0x0;
//   //   goIdle();
//   // }


  switch(state) {
    case STATE_IDLE:
      break;
    case STATE_SPIN:
      runMeltyBrain();//manage all of the sensors and predict our current heading

    //   if(en != 0xAA) {
    //     goIdle();
    //   }
    default:
      break;
  }
}



