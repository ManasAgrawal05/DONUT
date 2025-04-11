#include "meltybrain.h"

typedef struct melty_parameters_t {
  int translate_forback;              //RC_FORBACK_FORWARD, RC_FORBACK_BACKWARD, RC_FORBACK_NETURAL
  float throttle_percent;             //stores throttle percent
	unsigned long rotation_interval_us; //time for 1 rotation of robot
	unsigned long led_start;            //offset for beginning of LED beacon
	unsigned long led_stop;             //offset for end of LED beacon
	unsigned long motor_start_phase_1;  //time offset for when motor 1 turns on when translating forward (or motor 2 translating backwards)
	unsigned long motor_stop_phase_1;   //time offset for when motor 1 turns off when translating forward (or motor 2 translating backwards)
	unsigned long motor_start_phase_2;  //time offset for when motor 2 turns on when translating forward (or motor 1 translating backwards)
	unsigned long motor_stop_phase_2;   //time offset for when motor 2 turns on when translating forward (or motor 1 translating backwards)
  int steering_disabled;              //Prevents adjustment of left / right heading adjustment (used for configuration mode)
  int led_shimmer;                    //LED is shimmering to indicate something to the user
};

void setMotorSpeed(int motor, int spd) {
  spd = constrain(spd, -100, 100);//make sure our speed value is valid. This lets us be lazier elsewhere
  //apply a deadband
  if(spd < 5 && spd > -5) spd = 0;

  if(motor == motor1) spd *= -1;

  analogWrite(motor, map(spd, -100, 100, 64, 128));
}

// //updated the expected accelerometer reading for 0g 
// //assumes robot is not spinning when config mode is entered
// //value saved to EEPROM on config mode exit
// static void update_accel_zero_g_offset(){
//   int offset_samples = 200;
//   for (int accel_sample_loop = 0; accel_sample_loop < offset_samples; accel_sample_loop ++) {
//     accel_zero_g_offset += get_accel_force_g();
//   }
//   accel_zero_g_offset = accel_zero_g_offset / offset_samples;
// }

// // * Tanay Added verify please but this should be the most important method that gives us time between two motor HIGHs*
// //calculates time for this rotation of robot
// //robot is steered by increasing / decreasing rotation by factor relative to RC left / right position
// //ie - reducing rotation time estimate below actual results in shift of heading opposite the direction of rotation
// static float get_rotation_interval_ms() {
  
//   float radius_adjustment_factor = 0;

//   //don't adjust steering if disabled by config mode - or we are in RC deadzone
//   if (rc_get_is_lr_in_normal_deadzone() == false) {
//     radius_adjustment_factor = (float)(rc_get_leftright() / (float)NOMINAL_PULSE_RANGE) / LEFT_RIGHT_HEADING_CONTROL_DIVISOR;
//   }
  
//   float effective_radius_in_cm = accel_mount_radius_cm;
  
//   effective_radius_in_cm = effective_radius_in_cm + (effective_radius_in_cm * radius_adjustment_factor);

//   float rpm;
//   //use of absolute makes it so we don't need to worry about accel orientation
//   //calculate RPM from g's - derived from "G = 0.00001118 * r * RPM^2"
//   rpm = fabs(get_accel_force_g() - accel_zero_g_offset) * 89445.0f;
//   rpm = rpm / effective_radius_in_cm;
//   rpm = sqrt(rpm);

//   if (rpm > highest_rpm || highest_rpm == 0) highest_rpm = rpm;

//   float rotation_interval = (1.0f / rpm) * 60 * 1000;
//   return rotation_interval;
// }

// /* Added By Tanay : sets parameters in place to calculate translational interval offsets*/
// //Calculates all parameters need for a single rotation (motor timing, LED timing, etc.)
// //This entire section takes ~1300us on an Atmega32u4 (acceptable - fast enough to not have major impact on tracking accuracy)
// static struct melty_parameters_t get_melty_parameters(void) {

//   struct melty_parameters_t melty_parameters = {};

//   float led_offset_portion = led_offset_percent / 100.0f;

//   melty_parameters.throttle_percent = rc_get_throttle_percent() / 100.0f;

//   //by default motor_on_portion maps to thottle_percent input - but that can be altered
//   float motor_on_portion = melty_parameters.throttle_percent;

//   //changes motor_on_portion to fixed value if we are throttling via PWM if DYNAMIC_PWM_MOTOR_ON_PORTION is defined
// #ifdef DYNAMIC_PWM_MOTOR_ON_PORTION
//   if (THROTTLE_TYPE == DYNAMIC_PWM_THROTTLE) {
//     motor_on_portion = DYNAMIC_PWM_MOTOR_ON_PORTION;
//   }
// #endif 

//   float led_on_portion = melty_parameters.throttle_percent;  //LED width changed with throttle percent
//   if (led_on_portion < 0.10f) led_on_portion = 0.10f;
//   if (led_on_portion > 0.90f) led_on_portion = 0.90f;

//   melty_parameters.translate_forback = rc_get_forback();

//   //if we are in config mode - handle it (and disable steering if needed)
//   if (get_config_mode() == true) {
//     melty_parameters = handle_config_mode(melty_parameters);
//   }

//   melty_parameters.rotation_interval_us = get_rotation_interval_ms(melty_parameters.steering_disabled) * 1000;
  
//   //if under defined RPM - just try to spin up (motors on for full rotation)
//   if (melty_parameters.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US) motor_on_portion = 1;

//   //if we are too slow - don't even try to track heading
//   if (melty_parameters.rotation_interval_us > MAX_TRACKING_ROTATION_INTERVAL_US) {
//     melty_parameters.rotation_interval_us = MAX_TRACKING_ROTATION_INTERVAL_US;
//   }

//   unsigned long motor_on_us = motor_on_portion * melty_parameters.rotation_interval_us;
//   unsigned long led_on_us = led_on_portion * melty_parameters.rotation_interval_us;
//   unsigned long led_offset_us = led_offset_portion * melty_parameters.rotation_interval_us;

//   //starts LED on time at point in rotation so it's "centered" on led offset
//   if (led_on_us / 2 <= led_offset_us) {
//     melty_parameters.led_start = led_offset_us - (led_on_us / 2);
//   } else {
//     melty_parameters.led_start = (melty_parameters.rotation_interval_us + led_offset_us) - (led_on_us / 2);
//   }
  
//   melty_parameters.led_stop = melty_parameters.led_start + led_on_us;

//   //"wraps" led off time if it exceeds rotation length
//   if (melty_parameters.led_stop > melty_parameters.rotation_interval_us)
//     melty_parameters.led_stop = melty_parameters.led_stop - melty_parameters.rotation_interval_us;

//   //phase 1 timing: for motor_1 in forward translation or motor_2 in reverse
//   //motor "on" period is centered at the halfway point of the rotation cycle (6 o'clock)
//   melty_parameters.motor_start_phase_1 = (melty_parameters.rotation_interval_us / 2) - (motor_on_us / 2);
//   melty_parameters.motor_stop_phase_1 = melty_parameters.motor_start_phase_1 + motor_on_us;

//   //phase 2 timing: for motor_2 in forward translation or motor_1 in reverse
//   //180-degree phase shift relative to phase 1, centering the "on" period at the cycle's start/end (12 o'clock)
//   melty_parameters.motor_start_phase_2 = melty_parameters.rotation_interval_us - (motor_on_us / 2);
//   melty_parameters.motor_stop_phase_2 = motor_on_us / 2;

//   //if the battery voltage is low - shimmer the LED to let user know
// #ifdef BATTERY_ALERT_ENABLED
//   if (battery_voltage_low() == true) melty_parameters.led_shimmer = 1;
// #endif

//   return melty_parameters;
// }

// //rotates the robot once + handles translational drift
// //(repeat as needed)
// void spin_one_rotation(void) {

//   //-initial- assignment of melty parameters
//   static struct melty_parameters_t melty_parameters = get_melty_parameters();

//   //capture initial time stamp before rotation start (time performing accel sampling / floating point math is included)
//   unsigned long start_time = micros();
//   unsigned long time_spent_this_rotation_us = 0;

//   //tracking cycle count is needed to alternate cycles for non-translation (overflow is non-issue)
//   static unsigned long cycle_count = 0;
//   cycle_count++;

//   //the melty parameters are updated either at the beginning of the rotation - or the middle of the rotation (alternating each time)
//   //this is done so that any errors due to the ~1ms accel read / math cycle cancel out any effect on tracking / translational drift
//   int melty_parameter_update_time_offset_us = 0;
//   if (cycle_count % 2 == 1) melty_parameter_update_time_offset_us = melty_parameters.rotation_interval_us / 2;  
//   bool melty_parameters_updated_this_rotation = false;

//   //loop for one rotation of robot
//   while (time_spent_this_rotation_us < melty_parameters.rotation_interval_us) {

//     //update melty parameters if we haven't / update time has elapsed
//     if (melty_parameters_updated_this_rotation == false && time_spent_this_rotation_us > melty_parameter_update_time_offset_us) { 
//       melty_parameters = get_melty_parameters();
//       melty_parameters_updated_this_rotation = true;
//     }

//     //if translation direction is RC_FORBACK_NEUTRAL - robot cycles between forward and reverse translation for net zero translation
//     //if motor 2 (or motor 1) is not present - control sequence remains identical (signal still generated for non-connected motor)

//     //translate forward
//     if (melty_parameters.translate_forback == RC_FORBACK_FORWARD || (melty_parameters.translate_forback == RC_FORBACK_NEUTRAL && cycle_count % 2 == 0)) {
//       translate_forward(melty_parameters, time_spent_this_rotation_us);
//     }

//     //translate backward
//     if (melty_parameters.translate_forback == RC_FORBACK_BACKWARD || (melty_parameters.translate_forback == RC_FORBACK_NEUTRAL && cycle_count % 2 == 1)) {
//       translate_backward(melty_parameters, time_spent_this_rotation_us);
//     }

//     //displays heading LED at correct location
//     update_heading_led(melty_parameters, time_spent_this_rotation_us);

//     time_spent_this_rotation_us = micros() - start_time;

//   }

// }

void runMeltyBrain() {
//   unsigned long currTime = micros();//record the start of this iteration

  
  runAccelerometer();
  

  
//   //if the accelerometer is active, apply the trim
  
  angle = (accelAngle + accelTrim) % 360;
  

  //MOTOR COMMAND
  //first check if the melty throttle is high enough for translation
  // if(meltyThrottle > 10) {
  //   //calculate the distance between the current heading and the commanded direction
  //   int16_t diff = 180 - abs(abs(meltyAngle - angle) - 180);

  //   //now check if we are pointed towards the commanded direction or opposite or neither
  //   if(diff < 90) {
  //     //we are pointing towards the commanded heading, forward pulse
  //     setMotorSpeed(motor1, (1-flip*2)*(max(throt-20,0)));
  //     setMotorSpeed(motor2, (flip*2-1)*(throt));
  //   } else {
  //     //we are pointing opposite the commanded heading, reverse pulse
  //     setMotorSpeed(motor1, (1-flip*2)*(throt));
  //     setMotorSpeed(motor2, (flip*2-1)*(max(throt-20,0)));
  //   }
  // } else {
  //   //if we aren't translating, just run the motors at the throttle speed
  //   setMotorSpeed(motor1, (1-flip*2)*throt);
  //   setMotorSpeed(motor2, (flip*2-1)*throt);
  // }
}

