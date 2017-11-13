// This example shows how to make a Balboa balance on its two
// wheels and drive around while balancing.
//
// To run this demo, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino
//
// To use this demo, place the robot on the ground with the
// circuit board facing up, and then turn it on.  Be careful to
// not move the robot for a few seconds after powering it on,
// because that is when the gyro is calibrated.  During the gyro
// calibration, the red LED is lit.  After the red LED turns off,
// turn the robot so that it is standing up.  It will detect that
// you have turned it and start balancing.
//
// Alternatively, you can press the A button while the robot is
// lying down and it will try to use its motors to kick up into
// the balancing position.
//
// This demo is tuned for the 50:1 high-power gearmotor with
// carbon brushes, 45:21 plastic gears, and 80mm wheels; you will
// need to adjust the parameters in Balance.h for your robot.
//
// After you have gotten the robot balance well, you can
// uncomment some lines in loop() to make it drive around and
// play a song.

#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"

#define METERS_PER_CLICK 3.141592*80.0*(1/1000.0)/12.0/(162.5)
#define MOTOR_MAX 300
#define MAX_SPEED 0.75  // m/s
#define FORTY_FIVE_DEGREES_IN_RADIANS 0.78

extern int32_t angle_accum;
extern int32_t speedLeft;
extern int32_t driveLeft;
extern int32_t distanceRight;
extern int32_t speedRight;
extern int32_t distanceLeft;
extern int32_t distanceRight;

void balanceDoDriveTicks();

extern int32_t displacement;
int32_t prev_displacement=0;

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;

uint32_t prev_time;
unsigned long startTime;

void setup()
{
  Serial.begin(9600);
  prev_time = 0;
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  angle_accum = 0;
  ledGreen(0);
  ledYellow(0);
  startTime = millis();
}

extern int16_t angle_prev;
int16_t start_flag = 0;
int16_t armed_flag = 0;
int16_t start_counter = 0;
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

void newBalanceUpdate()
{
  static uint32_t lastMillis;
  uint32_t ms = millis();

  if ((uint32_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate encoders and gyros
  balanceUpdateSensors();
 
   if (imu.a.x < 0)
  {
    lyingDown();
    isBalancingStatus = false;
  }
  else
  {
    isBalancingStatus = true;
  }
}


float testSpeed = 0;          // this is the desired motor speed 
int16_t limitCount = 0;
float errAccumL = 0;
float errAccumR = 0;
float angleErrorSum = 0; // Sum of error for integral control of angle
#define ERROR_CLEAR_INTERVAL 500
float timeSinceErrorClear = 0;
int lastVelSign = 1;

float Ri = -6166;
float Rp = -912;
float Kp = 117;
float Ki = -77;

void loop()
{
  uint32_t cur_time = 0;
  static uint32_t prev_print_time = 0;   // this variable is to control how often we print on the serial monitor
  static float angle_rad;                // this is the angle in radians
  static float angle_rad_accum = 0;      // this is the accumulated angle in radians
  static float del_theta = 0;
  static float error_ = 0;      // this is the accumulated velocity error in m/s
  static float error_left_accum = 0;      // this is the accumulated velocity error in m/s
  static float error_right_accum = 0;      // this is the accumulated velocity error in m/s

  cur_time = millis();                   // get the current time in miliseconds

  newBalanceUpdate();                    // run the sensor updates. this function checks if it has been 10 ms since the previous 
  
  if(angle > 3000 || angle < -3000)      // If angle is not within +- 3 degrees, reset counter that waits for start
  {
    start_counter = 0;
  }

  // every UPDATE_TIME_MS, check if angle is within +- 3 degrees and we havent set the start flag yet
  float delta_t = (cur_time - prev_time)/1000.0;

  // handle the case where this is the first time through the loop
  if (prev_time == 0) {
    delta_t = 0.01;
  }

  if(cur_time - prev_time > UPDATE_TIME_MS && angle > -3000 && angle < 3000 && !armed_flag)   
  {
    // increment the start counter
    start_counter++;
    // If the start counter is greater than 30, this means that the angle has been within +- 3 degrees for 0.3 seconds, then set the start_flag
    if(start_counter > 30)
    {
      armed_flag = 1;
      buzzer.playFrequency(DIV_BY_10 | 445, 1000, 15);
    }
  }

  angle_rad = ((float)angle)/1000/180*3.14159 - del_theta;

  if(cur_time - prev_time > UPDATE_TIME_MS && (angle < -3000 || angle > 3000) && armed_flag)   
  {
    start_flag = 1;
    armed_flag = 0;
  }

  // every UPDATE_TIME_MS, if the start_flag has been set, do the balancing
  if(cur_time - prev_time > UPDATE_TIME_MS && start_flag)
  {
    // set the previous time to the current time for the next run through the loop
    prev_time = cur_time;

    float vL = METERS_PER_CLICK*speedLeft/delta_t;
    float vR = METERS_PER_CLICK*speedRight/delta_t;

    // set PWM_left and PWM_right here
    float PWM_left;
    float PWM_right;

    // Bottom of pendulum control
    float angleError = -angle_rad;
    angleErrorSum += angleError*delta_t;
    float targetPWM = angleError*Rp + Ri*angleErrorSum;

    // Cruise (wheel speed) control
    errAccumL += vL*delta_t;
    errAccumR += vR*delta_t;
    float aveVel = (vL + vR)/2;
    if (fabs(aveVel) < 0.02) { // && millis() > timeSinceErrorClear + ERROR_CLEAR_INTERVAL) {
      errAccumL = 0;
      errAccumR = 0;
      timeSinceErrorClear = millis();
      Serial.println("Cleared errors");
    }
    PWM_left = targetPWM + (vL*Kp + Ki*errAccumL);
    PWM_right = targetPWM + (vR*Kp + Ki*errAccumR);

    float PWM_ave = (PWM_left + PWM_right) / 2.0;
    PWM_left = PWM_ave;
    PWM_right = PWM_ave;
    
    // if the robot is more than 45 degrees, shut down the motor
    if(start_flag && fabs(angle_rad) > FORTY_FIVE_DEGREES_IN_RADIANS) // TODO: this was set to angle < -0.78... I changd it to angle_rad
    {
      // reset the accumulated errors here
      start_flag = 0;   /// wait for restart
      prev_time = 0;
      motors.setSpeeds(0,0);
    } else if(start_flag) {
      motors.setSpeeds((int)PWM_left, (int)PWM_right);
    }

    
    bool shouldPrint = cur_time - prev_print_time > 105;
    if(shouldPrint)   // do the printing every 105 ms. Don't want to do it for an integer multiple of 10ms to not hog the processor
    {
  //  Serial.print("/Angle (rad):");
      Serial.print(cur_time - startTime);
      Serial.print("\t");
      Serial.print(angle_rad, 5);   
      Serial.print("\t");
      Serial.print(vL);
      Serial.print("\t");
      Serial.print(vR);
      Serial.print("\t");
      Serial.print(angleErrorSum);
      Serial.print("\t");
      Serial.print(errAccumL);
      Serial.print("\t");
      Serial.println(errAccumR);
      prev_print_time = cur_time;
    }

  }

  // kill switch
  if (buttonA.getSingleDebouncedPress())
  {
      motors.setSpeeds(0,0);
      while(!buttonA.getSingleDebouncedPress());
  }
}
