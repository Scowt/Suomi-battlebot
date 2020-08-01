/*  
//  This file is to hold any preprocessor directives and general code foundations which were 
//  at the beginning of main.cpp, mainly to free up some real estate, while at the same time
//  making it easier to read what is going on via a less monolithic approach.
*/

#ifndef _PREPROC_H
#define _PREPROC_H
#include <Arduino.h>

//The below turn code features on. Comment out the line to disable.
//#define DEBUG         //turns on various Serial printfs, leds
#define INFRARED      //edge detection
//#define ADCBUTTONS  //Used if microprocessor runs out of digital io
//#define L298N       //used if I need to change motor driver board
#define LASERS        //Time of Flight sensor. The alternative was Ultrasonic, but these work much better.
#define IMU           //Inertial Measurement Unit - senses tilt of machine

//define physical locations - refer to https://www.pjrc.com/teensy/card8a_rev2.png
#define MOTOR_LEFT_EN 0
#define MOTOR_LEFT_IN_A 1
#define MOTOR_LEFT_IN_B 2
#define MOTOR_LEFT_PWM 3

#define MOTOR_RIGHT_EN 4
#define MOTOR_RIGHT_IN_A 5
#define MOTOR_RIGHT_IN_B 6
#define MOTOR_RIGHT_PWM 7

#define DEBUG1 8
#define DEBUG2 9
#define DEBUG3 10
#define DEBUG4 11 
#define DEBUG5 12
#define SPARE1 13
#define DECOY 14
#define IMU_ADO 15
#define I2C_SCL 16
#define I2C_SDA 17
#define PROX_REARRIGHT 19
#define PROX_REARLEFT 18 
#define PROX_LEFT_SIDE 20
#define PROX_RIGHT_SIDE 21
#define PROX_FRONT_LEFT 22
#define PROX_FRONT_RIGHT 23
#define DEBUG6 24 
#define DEBUG7 25
#define SWITCHA 26
#define SWITCHB 27
#define SWITCHC 28
#define SWITCHD 29
#define MOTOR_RIGHT_CS 33 //was 33
#define MOTOR_LEFT_CS 34  //was 34
#define CURRENTSENSE 35
#define IR_REARLEFT 36
#define IR_REARRIGHT 37
#define IR_FRONTRIGHT 38  
#define IR_FRONTLEFT 39

#define I2C_ADDR_MPU6050 104

//Possible machine states
#define NOINPUT 0
#define WAIT 1
#define EVADE 2
#define ATTACK 4
#define FULLSUMO 8
#define CURRENTLIMIT 16

//Possible movements for the vehicle
#define MOTION_FORWARD 1
#define MOTION_ROTATECCW 2
#define MOTION_VEERLEFT 3
#define MOTION_ROTATECW 4
#define MOTION_REVERSE 5
#define MOTION_REVERSE_VEERLEFT 6
#define MOTION_VEERRIGHT 7
#define MOTION_REVERSE_VEERRIGHT 8
#define MOTION_STOPPED 9
#define MOTION_COAST 10

//Speeds
#define SPEED_RAMMING 255 
#define SPEED_SEARCHING 50  //was 100, then 80, then 60
#define SPEED_AVOID 110     //was 150
#define SPEED_CHASE 160 
#define SPEED_EVADE 20      //was 70
#define SPEED_COAST 0       //was 0 
#define SPEED_MINIMUM 20    //was 40

#define SPEED_FACTOR 40 //small number means high acceleration, and higher probability of tilting.

//Miscellaneous Magic Numbers
#define MAX_DISTANCE_MM 1500  //Longest valid distance to sensed object; anything further we ignore
#define EVADE_DISTANCE  250 //Distance we wish to stay away from the opponent
#define MAX_TILT_Y 5
#define MAX_TILT_Z 4
#define SHIFTUP 30
#define SIMPLIFYFACTOR 1000

#ifdef INFRARED
  #define THRESHOLD_IR_FL 20  //was 110
  #define THRESHOLD_IR_FR 20
  #define THRESHOLD_IR_RL 20  //was 220
  #define THRESHOLD_IR_RR 20
#endif

//define sensor struct
struct bSensor
{
  bool Input;
  int16_t Raw;
  uint32_t Time;    //to store the time the Input was set
};
struct iSensor
{
  int16_t Input;
  uint32_t Time;    //to store the time the Input was set
  uint32_t AuxTime; //to store a separate time value
};

struct CyclicSensors
{
  //Time will reflect the last time the switch or sensor was high
  bSensor SwitchA, SwitchB, SwitchC, SwitchD;
  bSensor IR_FrontLeft, IR_FrontRight, IR_RearLeft, IR_RearRight;
  bSensor ImuAccelX,ImuAccelY,ImuAccelZ,ImuTemp,ImuGyroX,ImuGyroY,ImuGyroZ;
  //Time should reflect the last time the sensor was a "sane" value - ie, between 0.1 and 1500mm
  iSensor ProxFrontLeft, ProxFrontRight, ProxRearLeft, ProxRearRight;
  iSensor CS_MotorRight, CS_MotorLeft,CS_All;
};
#endif