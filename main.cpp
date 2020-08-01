/*  
//  Code for the sumobot for ser300
//  Programming begun in earnest 29/3/19 with the addition of sensor readings.
//  Programming completed about half an hour prior to the competition, 24/5/19
*/

// Incorporating some code from "MPU-6050 Short Example Sketch" by Arduino User JohnChi (Public Domain)
//Incorporating general code advice from https://stackoverflow.com/questions/11373139/c-class-object-pointers-and-accessing-member-functions

#include <arduino.h>
#include "preproc.h"
#include "motors.h"
#include "readsensors.h"
#include "Decisions.h"
#include "Wire.h"
#include <VL53L0X.h>

extern CyclicSensors Sensors;
ADC *adc = new ADC(); // adc object for sensors

VL53L0X FrickenLaserFR;
VL53L0X FrickenLaserFL;
VL53L0X FrickenLaserRR;
VL53L0X FrickenLaserRL;
//VL53L0X *FrickenLaserPointers;

void setup()
{
  //setup the adc
  adc->setAveraging(0);  // set number of averages
  adc->setResolution(8); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_0);

  //adc->enableInterrupts(ADC_0);

  //Set direction for Output pins
  pinMode(MOTOR_LEFT_EN, OUTPUT);
  pinMode(MOTOR_LEFT_IN_B, OUTPUT);
  pinMode(MOTOR_LEFT_IN_A, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_EN, OUTPUT);
  pinMode(MOTOR_RIGHT_IN_B, OUTPUT);
  pinMode(MOTOR_RIGHT_IN_A, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(PROX_FRONT_LEFT, OUTPUT);
  pinMode(PROX_FRONT_RIGHT, OUTPUT);
  pinMode(PROX_REARLEFT, OUTPUT);
  pinMode(DEBUG1, OUTPUT);
  pinMode(DEBUG2, OUTPUT);
  pinMode(DEBUG3, OUTPUT);
  pinMode(DEBUG4, OUTPUT);
  pinMode(DEBUG5, OUTPUT);
  pinMode(DEBUG6, OUTPUT);
  pinMode(DEBUG7, OUTPUT);
  pinMode(PROX_FRONT_LEFT, OUTPUT);
  pinMode(PROX_FRONT_RIGHT, OUTPUT);
  pinMode(PROX_LEFT_SIDE, OUTPUT);
  pinMode(PROX_RIGHT_SIDE, OUTPUT);
  pinMode(PROX_REARLEFT, OUTPUT);
  pinMode(PROX_REARRIGHT, OUTPUT);

  //Set direction for Input pins
  pinMode(SWITCHA, INPUT);
  pinMode(SWITCHB, INPUT);
  pinMode(SWITCHC, INPUT);
  pinMode(SWITCHD, INPUT);
  pinMode(IR_FRONTLEFT, INPUT);
  pinMode(IR_FRONTRIGHT, INPUT);
  pinMode(IR_REARLEFT, INPUT);
  pinMode(IR_REARRIGHT, INPUT);
  pinMode(MOTOR_LEFT_CS, INPUT);
  pinMode(MOTOR_RIGHT_CS, INPUT);
  pinMode(CURRENTSENSE,INPUT);

  //Initialise any pins what need it
  analogWriteFrequency(MOTOR_LEFT_PWM, 400);  //Oddly, increasing this by a factor of 10 (to 4000) decreases performance markedly (likely switching losses)
  analogWriteFrequency(MOTOR_RIGHT_PWM, 400); //Whilst decreasing it by another factor of 10 (to 40) introduces odd behaviour in the motor controllers.
  digitalWriteFast(DEBUG1, LOW);
  digitalWriteFast(DEBUG3, LOW);
  digitalWriteFast(DEBUG4, LOW);
  digitalWriteFast(DEBUG5, LOW);
  digitalWriteFast(DEBUG6, LOW);
  digitalWriteFast(DEBUG7, LOW);
  digitalWriteFast(MOTOR_RIGHT_IN_A, LOW);
  digitalWriteFast(MOTOR_RIGHT_IN_B, LOW);
  digitalWriteFast(MOTOR_LEFT_IN_A, LOW);
  digitalWriteFast(MOTOR_LEFT_IN_B, LOW);
  digitalWriteFast(MOTOR_LEFT_EN, LOW);
  digitalWriteFast(MOTOR_RIGHT_EN, LOW);

#ifdef DEBUG
  Serial.begin(115200);
#endif

  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
#ifdef IMU
  Wire.beginTransmission(I2C_ADDR_MPU6050);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  #endif
#ifdef LASERS
  SetupLasers();  //This function will set the laser addresses and test communications
#endif
}

void loop()
{
  static uint8_t Mode = 0;   //Byte to store states in. 1 NoInput, 2 Wait, 4 Evade 8 Attack 16 FullSumo 32 Current Limit
  static long WaitTimeBegin; //Will store the time at which the full sumo button was pressed; used to check that five seconds has elapsed
  //#ifdef DEBUG
  //static int LoopCount = 0;   //the purpose of this #IFDEF is to give the robot a heartbeat
  //LoopCount++;                //This can be used to show that the robot is functioning properly,
  //if (LoopCount > 2000)       //and also as a measure of loop cycle speed.
  //{
    //LoopCount = 0;
    //static int debuglight = 0;
    //debuglight++;
    //if (debuglight > 2)
    {
      if (digitalReadFast(DEBUG7))
      {
       digitalWriteFast(DEBUG7, LOW);
      }
      else
      {
        digitalWriteFast(DEBUG7, HIGH);
      }
      //debuglight = 0;
    //}
    //The below lines were used during development to show various values

    //Serial.printf("Mode: %d, A: %d, B: %d, C: %d, D: %d, ", Mode, Sensors.SwitchA.Input,Sensors.SwitchB.Input,Sensors.SwitchC.Input,Sensors.SwitchD.Input);
    //Serial.printf("IR_FR: %d, IR_FL: %d, IR_RR: %d, IR_RL: %d, \n",Sensors.IR_FrontRight.Raw ,Sensors.IR_FrontLeft.Raw, Sensors.IR_RearRight.Raw, Sensors.IR_RearLeft.Raw);
    //Serial.printf("Prox_FR: %d, Prox_FL: %d, Prox_RR: %d, Prox_RL: %d\n", Sensors.ProxFrontRight.Input, Sensors.ProxFrontLeft.Input, Sensors.ProxRearRight.Input, Sensors.ProxRearLeft.Input);
    //Serial.printf("X: %d, Y: %d, Z: %d, Temp: %d ", Sensors.ImuAccelX.Input, Sensors.ImuAccelY.Input, Sensors.ImuAccelZ.Input, Sensors.ImuTemp.Input);
    //Serial.printf("Right_I: %d, Left_I: %d\n", Sensors.CS_MotorRight.Input, Sensors.CS_MotorLeft.Input);
  }
  //#endif

  //State Machine section - NoInput, wait state, evade, attack, full sumo.

  ReadSensors(Mode);
  switch (Mode)
  {
    case NOINPUT: 
    {
      digitalWriteFast(DEBUG2, LOW);  //these leds will be used to show state machine status
      digitalWriteFast(DEBUG3, LOW);
      Motor_Action(MOTION_STOPPED, SPEED_RAMMING,false);  //Ensure that there is no motion in this state.

      if (Sensors.SwitchA.Input) //Attack button
      {
        Mode = ATTACK; //Find the box and ram eeeeeeeeetttttttttt
      }
      if (Sensors.SwitchB.Input) //Evade button
      {
        Mode = EVADE; //try not to get hit by the box
      }
      if (Sensors.SwitchC.Input) //Full Sumo button
      {
        Mode = WAIT;
        WaitTimeBegin = millis();
      }
      if (Sensors.SwitchD.Input) //Stop Button
      {
          //No action required
      }
      break;
    }
    case WAIT:
    {
      if (Sensors.SwitchD.Input)  //Stop Button - cease all motion immediately
      {
        Mode = NOINPUT;
        break;
      }
      if (Sensors.SwitchC.Input) //Full Sumo button - Checked again so that the timer only starts when button released
      {
        digitalWriteFast(DEBUG2, HIGH); //indicates that the program has entered a waiting state (turned off by NoInput)
        Mode = WAIT;
        WaitTimeBegin = millis();
      }
      if (millis() - WaitTimeBegin > 5000) //at least 5 seconds has passed since the button was pressed
        Mode = FULLSUMO;  //GO GET IM
      break;
    }
    case EVADE:
    {
      if (Sensors.SwitchD.Input)  //Stop Button - cease all motion immediately
      {
        Mode = NOINPUT;
        break;
      }
      if (FindEnemy(Mode)==EVADE)
      {
        //The actual motion direction has been moved to the findenemy function.
        digitalWriteFast(DEBUG2, HIGH);
      }
      else
      {
        digitalWriteFast(DEBUG2, LOW);
      }
      break;
    }
    case ATTACK:
    {
      if (Sensors.SwitchD.Input)  //Stop Button - cease all motion immediately
      {
        Mode = NOINPUT;
        break;
      }
      uint8_t KkD=FindEnemy(Mode);  //Krush Kill Destroy - placeholder variable for debugging purposes
      if (KkD==CHASE)               //Motion has been taken care of within the FindEnemy function
      {
        digitalWriteFast(DEBUG2, HIGH);
      }
      else
      {
        digitalWriteFast(DEBUG2, LOW);
      }
      break;
    }
    case FULLSUMO:
    {
      if (Sensors.SwitchD.Input)  //Stop Button - cease all motion immediately
      {
        Mode = NOINPUT;
        break;
      }
      uint8_t KkD=FindEnemy(Mode);  //variable only used for debugging
      if (KkD==CHASE)               //Motion control has been moved to the FindEnemy function
      {
        digitalWriteFast(DEBUG3, HIGH);
      }
      else 
      {
        digitalWriteFast(DEBUG3, LOW);
      }
      break;
    }
    case CURRENTLIMIT:            //This function was not implemented. As it turns out, the current sense provided by the motor controller 
    {  
      if (Sensors.SwitchD.Input)  //required extra circuitry - in essence, a low pass filter, as the analog value of 0.13v/A was output at 
      {                           //the same pwm duty cycle as the motor is turned on. This does not allow the ADC to give an accurate reading.
        Mode = NOINPUT;           //This was only realised scant days prior to the competition, and so was a low priority...
        break;
      }
      Motor_Action(MOTION_STOPPED, SPEED_RAMMING,false);
      break;
    }
  }
}
