#ifndef _READSENSORS_H
    #define _READSENSORS_H
    #include <Arduino.h>
    #include <ADC.h>
    #include "preproc.h"
    #include "Wire.h"

    #ifdef ADCBUTTONS
        #define BUTTON_VALUE_A 16   //A=0.20813=16
        #define BUTTON_VALUE_B 32   //B=.41625v=32
        #define BUTTON_VALUE_C 64   //C=.8325v=65
        #define BUTTON_VALUE_D 68   //D=1.665v=128
        #define BUTTON_THRESHOLD 4
    #endif

#ifdef LASERS
    #include <VL53L0X.h>
    extern VL53L0X FrickenLaserFR;
    extern VL53L0X FrickenLaserFL;
    extern VL53L0X FrickenLaserRR;
    extern VL53L0X FrickenLaserRL;
    #define LASER_READ_TIME 20   //time required for ToF sensors to finish a reading. At least this much time must have elapsed since our last read
    void SetupLasers();
#endif
    extern ADC *adc; 
    void echoCheck();
    void ReadSensors(uint8_t ModeSelected);
    void readLaser(VL53L0X *FrickenLaser, iSensor *ToFSensor);
    bool BringUpLaser(uint8_t Port, VL53L0X *FrickenLaser);
#endif