/*
//  This file is split away from the rest to replicate the function block of "read sensors"
//  and also try and keep the reading of inputs away from the main code for a bit of modularity.
//  This file should only contain code which involves reading from our environment; 
//anything else, whether setup or processing of inputs, can probably go elsewhere.
*/
#include "readsensors.h"

void SetupLasers()  //Sets up communication with all lasers
{
  uint8_t SensorSetup = 0;
  for (int i = 0; i < 5; i++)                 //Should ensure that all lasers are set up correctly.
  {                                           //This code should be tested by deliberately disabling a sensor
    SensorSetup = 0;                          //This is a flag signalling that something wasn't quite right and we need to try again
    digitalWriteFast(PROX_FRONT_RIGHT, LOW);  //Set all XSHUT pins low (will disable the chips reading the i2c bus)
    digitalWriteFast(PROX_FRONT_LEFT, LOW);
    digitalWriteFast(PROX_REARRIGHT, LOW);
    digitalWriteFast(PROX_REARLEFT, LOW);
    delay(500);                               //Ensure that the sensors have shut down
    SensorSetup = SensorSetup + BringUpLaser(PROX_FRONT_RIGHT, &FrickenLaserFR);  //If an error has occurred, the flag will no longer = 0
    SensorSetup = SensorSetup + BringUpLaser(PROX_FRONT_LEFT, &FrickenLaserFL);
    SensorSetup = SensorSetup + BringUpLaser(PROX_REARRIGHT, &FrickenLaserRR);
    SensorSetup = SensorSetup + BringUpLaser(PROX_REARLEFT, &FrickenLaserRL);
    if (SensorSetup == 0)
    {
      FrickenLaserFR.startContinuous();   //Continuous reading should allow me to retrieve a new sensor reading reasonably quickly
      FrickenLaserFL.startContinuous();   //instead of having to wait for a return signal every read
      FrickenLaserRL.startContinuous();
      FrickenLaserRR.startContinuous();
      break;
    }
  }
}

bool BringUpLaser(uint8_t Port, VL53L0X *FrickenLaser) //passed a pin and a sensor object, returns 0 if succesful, 1 if not
{
  bool SensorSetup = 0;
  digitalWriteFast(Port, HIGH);                       //Enables sensor. 
  delay(150);                                         //The delays prevent error setting addresses, ensuring the chip has settled
  FrickenLaser->init(false);                          //Initialises sensor
  delay(150);                                         
  FrickenLaser->setTimeout(250);                      //Timeout in ms to realise the sensor is not replying
  FrickenLaser->setMeasurementTimingBudget(20000);    //Time in us for 1 measurement; 20ms is minimum with the driver I am using
  FrickenLaser->setAddress(Port);                     //Change the I2C address of the sensor
  delay(10);
  Wire.beginTransmission(Port);                       //Start a transmission to the new address
  uint8_t error = Wire.endTransmission();             //End transmission, obtaining an error code
  if (error == 0)                                     //No error, we're all good here
  {
  #ifdef DEBUG
    Serial.printf("Successfully changed address to %d\n", Port);
  #endif
  }
  else
  {                                                   //Ruh Roh
  #ifdef DEBUG
    Serial.printf("Could not change address to %d - error id %d\n", Port, error);
  #endif
    SensorSetup = 1;                                  //Set error state to true:(
  }
  return SensorSetup;                                 //return error state
}




void ReadSensors(uint8_t ModeSelected)  //read all sensors.
{ /*
  //This function is intended to run at the start of each processor cycle.
  //It should read input from all sensors, and save their current values.
  //It should also save the time each sensor last received SANE values 
  //(ie, between 1 and 2000mm for a Time Of Flight sensor)
  */

  extern CyclicSensors Sensors;

#ifdef ADCBUTTONS //This IFDEF was never used; I have left it in here because I thought the idea was interesting.
                  //In essence, if I ran out of IO I was going to use a resistor network to read 4 or more pushbuttons off one analog pin.

  //TODO I have not confirmed the thresholds are correct. I also need to confirm the logic, I've likely got the arithmetic wrong.

  //TODO no masking or debouncing has been implemented. A hardware based debounce is unsatisfactory due to the ADC,
  // although I could use the ADC 'average' function.

  int ButtonInput = analogRead(SWITCHA);
  if (ButtonInput < (BUTTON_VALUE_A - BUTTON_THRESHOLD))
  {
    //There are no buttons pushed at this moment in time.
    Sensors.SwitchA.Input = 0;
    Sensors.SwitchB.Input = 0;
    Sensors.SwitchC.Input = 0;
    Sensors.SwitchD.Input = 0;
  }
  if ((ButtonInput - BUTTON_VALUE_A) < BUTTON_THRESHOLD)
  {
    //ie, button input should be 16, +/-4; in the range 12-20.
    Sensors.SwitchA.Time = millis();            //Store the time at which this button was pressed
    ButtonInput = ButtonInput - BUTTON_VALUE_A; //easier to check the next button
  }
  if ((ButtonInput - BUTTON_VALUE_B) < (2 * BUTTON_THRESHOLD))
  {
    //ie, button input should be 32, +/-8; in the range 24-40. (Threshold increases because uncertainty error is additive)
    Sensors.SwitchB.Time = millis();            //Store the time at which this button was pressed
    ButtonInput = ButtonInput - BUTTON_VALUE_B; //easier to check the next button
  }
  if ((ButtonInput - BUTTON_VALUE_C) < (3 * BUTTON_THRESHOLD))
  {
    //ie, button input should be 64, +/-16; in the range 48-80.
    Sensors.SwitchC.Time = millis();            //Store the time at which this button was pressed
    ButtonInput = ButtonInput - BUTTON_VALUE_C; //easier to check the next button
  }
  if ((ButtonInput - BUTTON_VALUE_D) < (4 * BUTTON_THRESHOLD))
  {
    //ie, button input should be 128, +/-32; in the range 96-160.
    Sensors.SwitchD.Time = millis(); //Store the time at which this button was pressed
  }
#else //we have enough pins so the above is a moot point
  Sensors.SwitchA.Input = digitalReadFast(SWITCHA);
  Sensors.SwitchB.Input = digitalReadFast(SWITCHB);
  Sensors.SwitchC.Input = digitalReadFast(SWITCHC);
  Sensors.SwitchD.Input = digitalReadFast(SWITCHD);
#endif
  if (Sensors.SwitchA.Input)          //If the button is pushed
  {
    Sensors.SwitchA.Time = millis();  //save the current time.
  }
  if (Sensors.SwitchB.Input)
  {
    Sensors.SwitchB.Time = millis();
  }
  if (Sensors.SwitchC.Input)
  {
    Sensors.SwitchC.Time = millis();
  }
  if (Sensors.SwitchD.Input)
  {
    Sensors.SwitchD.Time = millis();
  }

  //Reading IR sensors.
  Sensors.IR_FrontLeft.Raw = adc->analogRead(IR_FRONTLEFT);
  Sensors.IR_FrontRight.Raw = adc->analogRead(IR_FRONTRIGHT);
  Sensors.IR_RearLeft.Raw = adc->analogRead(IR_REARLEFT);
  Sensors.IR_RearRight.Raw = adc->analogRead(IR_REARRIGHT);

  static uint8_t Black_FL = 0;    //to capture the 'at rest' values of the analog sensors
  static uint8_t Black_FR = 0;    //This was to account for a hardware bug whereby varying light
  static uint8_t Black_RL = 0;    //levels between runs would result in completely different behaviour of the IR sensors .
  static uint8_t Black_RR = 0;
  static int8_t Black_IMUY = 0;   //Note - these IMU values Can be negative.
  static int8_t Black_IMUZ = 0;


  if ((Black_FL - Sensors.IR_FrontLeft.Raw) < THRESHOLD_IR_FL)  
  {
    Sensors.IR_FrontLeft.Input = false;   //If the current IR level is much the same as the 'at rest' value 
  }
  else
  {
    Sensors.IR_FrontLeft.Input = true;    //the current IR level is very different to the at rest value
    Sensors.IR_FrontLeft.Time = millis(); //saving current time. (ensures corrective action will continue once fault cleared)
  }

  if ((Black_FR - Sensors.IR_FrontRight.Raw) < THRESHOLD_IR_FR)
  {
    Sensors.IR_FrontRight.Input = false;
  }
  else
  {
    Sensors.IR_FrontRight.Input = true;
    Sensors.IR_FrontRight.Time = millis();
  }

  if ((Black_RL - Sensors.IR_RearLeft.Raw) < THRESHOLD_IR_RL)
  {
    Sensors.IR_RearLeft.Input = false;
  }
  else
  {
    Sensors.IR_RearLeft.Input = true;
    Sensors.IR_RearLeft.Time = millis();
  }

  if ((Black_RR - Sensors.IR_RearRight.Raw) < THRESHOLD_IR_RR)
  {
    Sensors.IR_RearRight.Input = false;
  }
  else
  {
    Sensors.IR_RearRight.Input = true;
    Sensors.IR_RearRight.Time = millis();
  }


  static uint8_t SelectLaser = 0;     //I do not wish to read every laser every loop; the sensor requires at least 20ms 
                                      //between reads, else the driver hangs and does not return control to my program in a timely fashion.
  SelectLaser++;
  switch (SelectLaser)
  {
    case 1:
      break;    //do not read any ToF sensors
    case 2:
    {
      static uint32_t TimeFR = millis();          //saves the time values between function calls
      if ((millis() - TimeFR) > LASER_READ_TIME)  //check if at least 20ms has elapsed since we last checked this particular sensor
      { 
        TimeFR = millis();                        //capture current time
        readLaser(&FrickenLaserFR, &Sensors.ProxFrontRight);  //read current sensor value
      }
      break;
    }
    case 3:     //do not read any ToF sensors
      break;
    case 4:
    {
      static uint32_t TimeFL = millis();
      if ((millis() - TimeFL) > LASER_READ_TIME)
      {
        TimeFL = millis();
        readLaser(&FrickenLaserFL, &Sensors.ProxFrontLeft);
      }
      break;
    }
    case 5:     //do not read any ToF sensors
      break;
    case 6:
    {
      static uint32_t TimeRL = millis();
      if ((millis() - TimeRL) > LASER_READ_TIME)
      {
        TimeRL = millis();
        readLaser(&FrickenLaserRL, &Sensors.ProxRearLeft);
      }
      break;
    }
    case 7:     //do not read any ToF sensors
      break;
    case 8:
    {
      static uint32_t TimeRR = millis();
      if ((millis() - TimeRR) > LASER_READ_TIME)
      {
        TimeRR = millis();
        readLaser(&FrickenLaserRR, &Sensors.ProxRearRight);
      }
      break;
    }
    default:      //do not read any ToF sensors
    {
      SelectLaser = 0;
    }
  }
  if(!(ModeSelected==EVADE))
  {
    //In the case that we are not in Evade mode, we should set the Auxillary time of each sensor arbitrarily high to avoid false triggers.
    Sensors.ProxRearRight.AuxTime=UINT32_MAX;   //this value would be very unlikely to ever arise in normal use.
    Sensors.ProxRearLeft.AuxTime=UINT32_MAX;
    Sensors.ProxFrontLeft.AuxTime=UINT32_MAX;
    Sensors.ProxFrontRight.AuxTime=UINT32_MAX;
  }

#ifdef IMU
  static uint16_t ReadMPU=0;
  ReadMPU++;
  if(ReadMPU>1000)  //an effort to avoid loading up the I2C bus; currently unsure if it was the I2C bus or the IMU at fault,
  {                 //but a free running IMU sensor read slowed the loop frequency from 6khz to ~500hz
    ReadMPU=0;
    Wire.beginTransmission(I2C_ADDR_MPU6050);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.requestFrom(I2C_ADDR_MPU6050, 6, true); // request a total of 6 registers
    //These variables are of int datatype.
    Sensors.ImuAccelX.Raw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    Sensors.ImuAccelY.Raw = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    Sensors.ImuAccelZ.Raw = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Wire.endTransmission(true);                             //ensures we don't leave the I2C bus hanging. 'Just in case'
    Sensors.ImuAccelX.Time=millis();                        
    Sensors.ImuAccelY.Time=millis();                        //Save current time
    Sensors.ImuAccelZ.Time=millis();
    Sensors.ImuAccelX.Raw =(Sensors.ImuAccelX.Raw/SIMPLIFYFACTOR)+SHIFTUP;        //I am dividing the Accel values by ~1000 to reduce significant digits 
    Sensors.ImuAccelY.Raw =(Sensors.ImuAccelY.Raw/SIMPLIFYFACTOR)+SHIFTUP;        //and also adding ~30 to all values to reduce code complexity
    Sensors.ImuAccelZ.Raw =(Sensors.ImuAccelZ.Raw/SIMPLIFYFACTOR)+SHIFTUP;        //the actual values are arbitrary - I just want to know if there has been a large shift in robot plane
  }
#endif
#ifndef L298N //Using the motor drivers inbuilt current sense
  static uint16_t ReadCurrentSense=0;
  ReadCurrentSense++;
if(ReadCurrentSense>1000)   //I don't particularly care what the current is in every loop, more what the trend is
{
  ReadCurrentSense=0;
  Sensors.CS_MotorRight.Input=adc->analogRead(MOTOR_RIGHT_CS);
  Sensors.CS_MotorRight.Time=millis();
  Sensors.CS_MotorLeft.Input=adc->analogRead(MOTOR_LEFT_CS);
  Sensors.CS_MotorLeft.Time=millis();
  //Sensors.CS_All.Input=adc->analogRead(CURRENTSENSE);   //allowance was made on the pcb for an off board current sense module.
  //Sensors.CS_All.Time=millis();                         //This module was not wired in due to time constraints
}
#endif
  if (ModeSelected==NOINPUT||ModeSelected==WAIT)  //if we are in Wait or NoInput mode, allow for changing light conditions (aka, dont finalise IR thresholds)
  {
    Black_FL = Sensors.IR_FrontLeft.Raw;
    Black_FR = Sensors.IR_FrontRight.Raw;
    Black_RL = Sensors.IR_RearLeft.Raw;
    Black_RR = Sensors.IR_RearRight.Raw;
    Black_IMUY = Sensors.ImuAccelY.Raw;
    Black_IMUZ = Sensors.ImuAccelZ.Raw;
  }
  if(Sensors.ImuAccelY.Raw-Black_IMUY>MAX_TILT_Y)
  {
    Sensors.ImuAccelY.Input=1;
  }
  else
  {
    Sensors.ImuAccelY.Input=0;
  }
  if(Sensors.ImuAccelZ.Raw-Black_IMUZ>MAX_TILT_Z)
  {
    Sensors.ImuAccelZ.Input=1;
  }
  else
  {
    Sensors.ImuAccelZ.Input=0;
  }
  //The below were often used throughout development for debugging purposes.
  //Serial.printf("FL: %d & %d, Black: %d, FR: %d & %d, Black: %d",Sensors.IR_FrontLeft.Input, Sensors.IR_FrontLeft.Raw,Black_FL,Sensors.IR_FrontRight.Input, Sensors.IR_FrontRight.Raw,Black_FR);
  //Serial.printf("RL: %d & %d, Black: %d, RR: %d & %d, Black: %d\n",Sensors.IR_RearLeft.Input, Sensors.IR_RearLeft.Raw,Black_RL,Sensors.IR_RearRight.Input, Sensors.IR_RearRight.Raw,Black_RR);
  //Serial.printf("Y: %d & %d & %d, Z: %d & %d & %d\n",Sensors.ImuAccelY.Input,Sensors.ImuAccelY.Raw,Black_IMUY, Sensors.ImuAccelZ.Input, Sensors.ImuAccelZ.Raw,Black_IMUZ);
}


void readLaser(VL53L0X *FrickenLaser, iSensor *ToFSensor) //accepts pointer to laser sensor and pointer to a struct, saves data to struct
{
  uint16_t Distance = FrickenLaser->readRangeContinuousMillimeters();
  if ((Distance > 0) && (Distance < MAX_DISTANCE_MM)) //valid sensor reading
  {
    ToFSensor->Input = Distance;
    ToFSensor->Time = millis();
    if(Distance <EVADE_DISTANCE)  
    {
      ToFSensor->AuxTime=millis(); //saves the time, for use if in evade mode - how long ago was the object closer than EVADE_DISTANCE
    }
  }
  else
  {
    ToFSensor->Input = 0;
  }
}