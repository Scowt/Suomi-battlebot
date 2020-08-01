#ifndef _DECISIONS_H
#define _DECISIONS_H
#include "motors.h"
#include <Arduino.h>
#include <VL53L0X.h>
#include "preproc.h"

//define magic numbers
#define ENEMY_DETECTED_MS_AGO 2
#define ENEMY_DETECTED_WHILE_AGO 4
#define ENEMY_AT_REAR_MS_AGO 1000
#define MAX_CHARS_COST 10
#define NUM_MOTOR_COSTS 9
#define FRONT_EDGE_TIME  120//time in ms to keep moving away from edge - Front IR sensors
#define REAR_EDGE_TIME 100 //time in ms to keep moving away from edge - Rear IR sensors 
#define COAST_TIME 10
#define FIRSTSPIN 2000
#define STRAIGHTRUNTIME 2000
#define KILL_DISTANCE_MM 200
#define RAMTIME 10000

//return values
#define KILL 0
#define CHASE 1
#define RECENTLOCK 2
#define RECENTLOCK_LEFT 3
#define RECENTLOCK_RIGHT 4
#define HISTORICALLOCK 5
#define HESBEHINDYOU 6
#define CURRENTLEFT 7
#define CURRENTRIGHT 8
#define EVADESTOP 9
#define NOLOCK 10

//Placeholders
#define CLOCKWISE 0
#define COUNTERCLOCKWISE 1

uint8_t EdgeAvoid(uint8_t OriginalMotion);
uint8_t IsEnemyVisible(uint8_t Mode);
uint8_t FindEnemy(uint8_t Mode);

struct Costs
{
    uint8_t CostId, CostValue;
    char CostName[MAX_CHARS_COST];
};
#endif