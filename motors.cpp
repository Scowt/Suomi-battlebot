/*
//  This file is to hold the code dedicated to the implementation of motor control.
//  Ideally, no smarts should take place here; It should receive instructions, carry them out.
//  Directional States the robot can be in:
//  S1  L   forward     R   forward |   Forward: no object; attack; opponent behind; edge behind
//  S2  L   back        R   forward |   Rotate Anticlockwise: Searching for opponent
//  S3  L   off         R   forward |   Veering leftwards: Front Right edge detect has been on recently
//  S4  L   forward     R   back    |   Rotate Clockwise: Searching for opponent?
//  S5  L   back        R   back    |   Reverse: Both front edge detect have been on recently
//  S6  L   off         R   back    |   reverse veering Left: ???
//  S7  L   forward     R   off     |   Veering Rightwards: Front Left edge detect has been on recently
//  S8  L   back        R   off     |   reverse veering Right: ???
//  S9  L   off         R   off     |   Stopped: overcurrent, undervoltage, waiting for input
*/
#include "motors.h"

void Motor_Action(int8_t Action, uint16_t Speed,bool UrgentAction)
{
  static bool MovingForward=0;  //Robot already in forward motion; used for acceleration ramping
  static uint32_t LoopCount;    //Number of loops since acceleration started
  switch(Action)
  {
    case MOTION_REVERSE:
    {
      MovingForward=false;  //clear the move forward flag
      Motor_Right_Set(MOTION_REVERSE, Speed);
      Motor_Left_Set(MOTION_REVERSE, Speed);  
      break;
    }
    case MOTION_STOPPED:
    {
      MovingForward=false;  //clear the move forward flag
      Motor_Right_Set(MOTION_STOPPED, Speed);
      Motor_Left_Set(MOTION_STOPPED, Speed);
      break;
    }
    case MOTION_FORWARD:
    {
      if(UrgentAction)    //if immediate movement is required, don't bother with the ramped acceleration
      {
      Motor_Right_Set(MOTION_FORWARD, Speed);
      Motor_Left_Set(MOTION_FORWARD, Speed);
      break;
      }
      if(!MovingForward)                      //the last loop, we were not trying to move forward.
      {
        MovingForward=true;                   //but now we are so set this flag
        LoopCount=SPEED_MINIMUM*SPEED_FACTOR; //start the loop at this minimum value
      }
      if((LoopCount/SPEED_FACTOR)<Speed)      //still accelerating
      {
        LoopCount++;                          
        Speed=LoopCount/SPEED_FACTOR;         //scales the acceleration
      }      
      Motor_Right_Set(MOTION_FORWARD, Speed);
      Motor_Left_Set(MOTION_FORWARD, Speed);
      break;
    }
    case MOTION_COAST:
    {
      MovingForward=false;  //clear the move forward flag
      Motor_Right_Set(MOTION_COAST, Speed);
      Motor_Left_Set(MOTION_COAST, Speed);
      break;
    }
    case MOTION_ROTATECCW:
    {
      MovingForward=false;  //clear the move forward flag
      Motor_Right_Set(MOTION_FORWARD, Speed);
      Motor_Left_Set(MOTION_REVERSE, Speed);
      break;
    }
    case MOTION_ROTATECW:
    {
      MovingForward=false;  //clear the move forward flag
      Motor_Right_Set(MOTION_REVERSE, Speed);
      Motor_Left_Set(MOTION_FORWARD, Speed);
      break;
    }
    case MOTION_VEERLEFT:
    {      
      if(UrgentAction)
      {
        Motor_Right_Set(MOTION_FORWARD, Speed);
        Motor_Left_Set(MOTION_FORWARD, Speed/2);
        break;
      }
      if(!MovingForward)                      //the last loop, we were not trying to move forward.
      {
        MovingForward=true;                   //but now we are so set this flag
        LoopCount=SPEED_MINIMUM*SPEED_FACTOR; //start the loop at this minimum value
      }
      if((LoopCount/SPEED_FACTOR)<Speed)      //still accelerating
      {
        LoopCount++;                          
        Speed=LoopCount/SPEED_FACTOR;         //scales the acceleration
      }      
      Motor_Right_Set(MOTION_FORWARD, Speed);
      Motor_Left_Set(MOTION_FORWARD, Speed/2);      
      break;
    }
    case MOTION_VEERRIGHT:
    {   
      if(UrgentAction)
      {
      Motor_Right_Set(MOTION_FORWARD, Speed/2);
      Motor_Left_Set(MOTION_FORWARD, Speed);
      break;
      }
       if(!MovingForward)                      //the last loop, we were not trying to move forward.
      {
        MovingForward=true;                   //but now we are so set this flag
        LoopCount=SPEED_MINIMUM*SPEED_FACTOR; //start the loop at this minimum value
      }
      if((LoopCount/SPEED_FACTOR)<Speed)      //still accelerating
      {
        LoopCount++;                          
        Speed=LoopCount/SPEED_FACTOR;         //scales the acceleration
      }      
      Motor_Right_Set(MOTION_FORWARD, Speed/2);
      Motor_Left_Set(MOTION_FORWARD, Speed);      
      break;
    }
    case MOTION_REVERSE_VEERLEFT:
    { //away from front right
      MovingForward=false;  //clear the move forward flag
      Motor_Right_Set(MOTION_REVERSE,Speed/2);
      Motor_Left_Set(MOTION_REVERSE,Speed);
      break;
    }
    case MOTION_REVERSE_VEERRIGHT:
    {//away from front left
      MovingForward=false;  //clear the move forward flag
      Motor_Right_Set(MOTION_REVERSE,Speed);
      Motor_Left_Set(MOTION_REVERSE,Speed/2);
      break;
    }
  }
}

void Motor_Right_Set(int8_t Direction, uint16_t Speed)  //communicate with motor driver
                        //I am leaving these as two separate functions, as it would be overly complex to 
                        //create one function to replace them
{
  if (Direction == MOTION_REVERSE)
  {
    #ifndef L298N   //If L298N not defined, the motor driver settings used are those for the VNH2SP30
    digitalWriteFast(MOTOR_RIGHT_EN, HIGH);
    digitalWriteFast(MOTOR_RIGHT_IN_A, LOW); 
    digitalWriteFast(MOTOR_RIGHT_IN_B, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, Speed);
    #else //L298N code has been left in, just in case I have a motor driver incident on show day
    analogWrite(MOTOR_RIGHT_PWM, Speed);
    digitalWriteFast(MOTOR_RIGHT_IN_A, HIGH);
    digitalWriteFast(MOTOR_RIGHT_IN_B, LOW);
    #endif
  }
  if (Direction == MOTION_STOPPED)
  {
    #ifndef L298N
    digitalWriteFast(MOTOR_RIGHT_EN, LOW);
    digitalWriteFast(MOTOR_RIGHT_IN_A, HIGH);
    digitalWriteFast(MOTOR_RIGHT_IN_B, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, SPEED_MINIMUM);
    #else
    analogWrite(MOTOR_RIGHT_PWM, 0);
    digitalWriteFast(MOTOR_RIGHT_IN_A, LOW);
    digitalWriteFast(MOTOR_RIGHT_IN_B, LOW);
    #endif
  }
  if (Direction == MOTION_FORWARD)
  {
    #ifndef L298N
    digitalWriteFast(MOTOR_RIGHT_EN, HIGH);
    digitalWriteFast(MOTOR_RIGHT_IN_A, HIGH);
    digitalWriteFast(MOTOR_RIGHT_IN_B, LOW);
    analogWrite(MOTOR_RIGHT_PWM, Speed);
    #else
    analogWrite(MOTOR_RIGHT_PWM, Speed);
    digitalWriteFast(MOTOR_RIGHT_IN_A, LOW);
    digitalWriteFast(MOTOR_RIGHT_IN_B, HIGH);
    #endif
  }
  if (Direction == MOTION_COAST)
  {
    analogWrite(MOTOR_RIGHT_PWM, SPEED_MINIMUM ); //will act to slow down the motor without harshly braking it
  }
}

void Motor_Left_Set(int8_t Direction, uint16_t Speed)//communicate with motor driver
// *** comments will be the same as for Motor_Left_Set ***
                        //I am leaving these as two separate functions, as it would be overly complex to 
                        //create one function to replace them

{
  if (Direction == MOTION_REVERSE)
  {
    #ifndef L298N       //If L298N not defined, the motor driver settings used are those for the VNH2SP30
    digitalWriteFast(MOTOR_LEFT_EN, HIGH);
    digitalWriteFast(MOTOR_LEFT_IN_A, HIGH);
    digitalWriteFast(MOTOR_LEFT_IN_B, LOW);
    analogWrite(MOTOR_LEFT_PWM, Speed);
    #else               //L298N code has been left in, just in case I have a motor driver incident on show day
    analogWrite(MOTOR_LEFT_PWM, Speed);
    digitalWriteFast(MOTOR_LEFT_IN_A, LOW);
    digitalWriteFast(MOTOR_LEFT_IN_B, HIGH);
    #endif
  }
  if (Direction == MOTION_STOPPED)
  {
      #ifndef L298N
    digitalWriteFast(MOTOR_LEFT_EN, LOW);
    digitalWriteFast(MOTOR_LEFT_IN_A, HIGH);
    digitalWriteFast(MOTOR_LEFT_IN_B, HIGH);
    analogWrite(MOTOR_LEFT_PWM, SPEED_MINIMUM);
    #else
    analogWrite(MOTOR_LEFT_PWM, 0);
    digitalWriteFast(MOTOR_LEFT_IN_A, LOW);
    digitalWriteFast(MOTOR_LEFT_IN_B, LOW);
    #endif
  }
  if (Direction == MOTION_FORWARD)
  {
    #ifndef L298N
    digitalWriteFast(MOTOR_LEFT_EN, HIGH);
    digitalWriteFast(MOTOR_LEFT_IN_A, LOW);
    digitalWriteFast(MOTOR_LEFT_IN_B, HIGH);
    analogWrite(MOTOR_LEFT_PWM, Speed);
    #else
    analogWrite(MOTOR_LEFT_PWM, Speed);
    digitalWriteFast(MOTOR_LEFT_IN_A, HIGH);
    digitalWriteFast(MOTOR_LEFT_IN_B, LOW);
    #endif
  }
  if (Direction == MOTION_COAST)
  {
    analogWrite(MOTOR_LEFT_PWM, SPEED_MINIMUM);
  }
}