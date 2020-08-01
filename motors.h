#ifndef _MOTORS_H
    #define _MOTORS_H
    #include <Arduino.h>
    #include "preproc.h"
    void Motor_Right_Set(int8_t Direction, uint16_t Speed);
    void Motor_Left_Set(int8_t Direction, uint16_t Speed);
    void Motor_Action(int8_t Action, uint16_t Speed,bool UrgentAction);
#endif