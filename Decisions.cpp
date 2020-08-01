//This file contains the rudimentary AI for the robot.
//EdgeAvoid checks for the edge and assigns an action if required to move away from it,
//AvoidTilt checks for machine tilt and assigns a slower speed if required to reduce tilt,
//IsEnemyVisible checks the Time of Flight proximity sensors for objects and returns a recommended action
//FindEnemy ties the other functions together, and adds minor higher decision making.


#include "Decisions.h"
CyclicSensors Sensors;



uint8_t EdgeAvoid(uint8_t OriginalMotion)//Checks all IR sensors; if all ok, returns original motion value; if not ok, returns required motion to avoid going over the edge.
{
    // At present I have a high reliance on time - That is, I implicitly work out how far I have travelled based on elapsed time. This is subpar.
    //Given more time for shipping, I should like to use encoders on my motors.

    int MotionState = OriginalMotion;
    if (((millis() - Sensors.IR_FrontLeft.Time) < FRONT_EDGE_TIME ) && ((millis() - Sensors.IR_RearLeft.Time) < REAR_EDGE_TIME ) && ((millis() - Sensors.IR_FrontRight.Time) < FRONT_EDGE_TIME ) && ((millis() - Sensors.IR_RearRight.Time) < REAR_EDGE_TIME ))
    {   //All sensors are currently on; this should be impossible, and indicates a fault. Ignore the IR sensors :/
        MotionState = OriginalMotion;
        return MotionState;
    }
    if (((millis() - Sensors.IR_FrontLeft.Time) < FRONT_EDGE_TIME ) && (!((millis() - Sensors.IR_FrontRight.Time) < FRONT_EDGE_TIME )))
    {   //Front left and not front right
        //Code to move motors away from front left (Veering Rightwards in Reverse)
        MotionState = MOTION_REVERSE_VEERRIGHT;
    }

    if (((millis() - Sensors.IR_FrontRight.Time) < FRONT_EDGE_TIME ) && (!((millis() - Sensors.IR_FrontLeft.Time) < FRONT_EDGE_TIME )))
    {//front right and not front left
        //Code to move motors away from front Right (Veering Lefttwards in Reverse)
        MotionState = MOTION_REVERSE_VEERLEFT;
    }

    if (((millis() - Sensors.IR_FrontRight.Time) < FRONT_EDGE_TIME ) && ((millis() - Sensors.IR_FrontLeft.Time) < FRONT_EDGE_TIME ))
    {//Both front sensors are on, indicating I'm driving forwards off the board.
        if((millis() - Sensors.IR_FrontRight.Time) < (FRONT_EDGE_TIME-COAST_TIME) )
        {//Move backwards
            MotionState = MOTION_REVERSE;
        } 
        else
        {   //if the robot suddenly stops moving backwards, it can tilt. Coasting at the end lessens the moment around the motors.
            MotionState=MOTION_COAST;
        }
    }

    if (((millis() - Sensors.IR_RearRight.Time) < REAR_EDGE_TIME ) && ((millis() - Sensors.IR_RearLeft.Time) < REAR_EDGE_TIME ))
    {
        //Both rear facing IR sensors
        MotionState = MOTION_FORWARD;
    }

    if (((millis() - Sensors.IR_RearRight.Time) < REAR_EDGE_TIME ) && (!((millis() - Sensors.IR_RearLeft.Time) < REAR_EDGE_TIME )))
    {
        //rear right is on edge, rear left is not;
        MotionState = MOTION_VEERLEFT;
    }

    if (((millis() - Sensors.IR_RearLeft.Time) < REAR_EDGE_TIME ) && (!((millis() - Sensors.IR_RearRight.Time) < REAR_EDGE_TIME )))
    {
        //rear left is on edge, rear right is not;
        MotionState = MOTION_VEERRIGHT;
    }

    if (((millis() - Sensors.IR_FrontRight.Time) < FRONT_EDGE_TIME ) && ((millis() - Sensors.IR_RearRight.Time) < REAR_EDGE_TIME ))
    {   //Front Right and Rear Right
        MotionState = MOTION_REVERSE_VEERLEFT;
    }

    if (((millis() - Sensors.IR_FrontLeft.Time) < FRONT_EDGE_TIME ) && (((millis() - Sensors.IR_RearLeft.Time) < REAR_EDGE_TIME )))
    {   //Front Left and Rear Left
        MotionState = MOTION_REVERSE_VEERRIGHT;
    }
    return MotionState;
}

#ifdef IMU
uint8_t AvoidTilt(uint8_t OriginalSpeed)
{
    static bool StillTilted=0;      //flag - was the robot tilted the last time this function was called?
    static uint16_t LoopCount=0;    //number of loops robot has been tilted
    uint8_t Speed=0;                //Speed to return
    if(Sensors.ImuAccelY.Input)
    {                                               //If robot is tilted
        if(!StillTilted)
        {                                           //and this is the first time it has been tilted
            StillTilted=true;                       //set a flag
            LoopCount=OriginalSpeed*SPEED_FACTOR;   //take note of the original speed (allows a deceleration)
        }
        if((LoopCount/SPEED_FACTOR)>0)              //If robot is still moving, continue deceleration
        {
            LoopCount--;
            Speed=LoopCount/SPEED_FACTOR; 
        }  
        digitalWriteFast(DEBUG6,HIGH);              //Indicator light that robot is in tilt mode
        return Speed;
    }
    else 
    {
        StillTilted=false;                          //Robot is no longer tilted; reset the flag so that next time
        digitalWriteFast(DEBUG6,LOW);               // a tilt occurs, we begin deceleration afresh.
        return OriginalSpeed;
    }
}
#endif

uint8_t IsEnemyVisible(uint8_t Mode)
{
    iSensor *Left;
    iSensor *Right;
    iSensor *RearLeft;
    iSensor *RearRight;
    uint16_t CurrentMaxDistance = 0;
    if (Mode == ATTACK || Mode == FULLSUMO)
    {
        //Use the front sensors
        Left = &Sensors.ProxFrontLeft;
        Right = &Sensors.ProxFrontRight;
        RearLeft = &Sensors.ProxRearLeft;
        RearRight = &Sensors.ProxRearRight;
        CurrentMaxDistance = MAX_DISTANCE_MM;
    }
    else
    {
        //Use the rear sensors
        //Need to mirror Left vs Right, as the movement sections are based on FRONT FACING sensors
        Left = &Sensors.ProxRearRight;
        Right = &Sensors.ProxRearLeft;
        RearRight = &Sensors.ProxFrontLeft;
        RearLeft = &Sensors.ProxFrontRight;
        CurrentMaxDistance = EVADE_DISTANCE;
    }
    if ((Left->Input) && (Left->Input < CurrentMaxDistance))
    {
        digitalWriteFast(DEBUG4, HIGH);
    }
    else
    {
        digitalWriteFast(DEBUG4, LOW);
    }
    if ((Right->Input) && (Right->Input < CurrentMaxDistance))
    {
        digitalWriteFast(DEBUG1, HIGH);
    }
    else
    {
        digitalWriteFast(DEBUG1, LOW);
    }
    //The sensor values are set to 0 if the reading is out of bounds. In hindsight this could have been set arbitrarily high for cleaner code!
    if (((Left->Input) && (Left->Input < MAX_DISTANCE_MM)) && ((Right->Input) && (Right->Input < MAX_DISTANCE_MM)))
    {//If valid sensor values for left and right, and both left and right sensors report an object within range
        if (Mode == EVADE)
        {//evade mode modifies behaviour so that the robot will attempt to keep itself 250mm away from the object, with its rear facing the object at all times.
            if ((Left->Input >= EVADE_DISTANCE) && (Right->Input >= EVADE_DISTANCE))
            {   
                return EVADESTOP; //The box is right where we want it to be; in front of us, and further than Evade_Distance
            }
            else 
            {
                if ( ((Left->Input < EVADE_DISTANCE)||(millis()-Left->AuxTime<ENEMY_DETECTED_MS_AGO))  && ((Right->Input < EVADE_DISTANCE)||(millis()-Right->AuxTime<ENEMY_DETECTED_MS_AGO)) )
                {//if the object is closer than ~250mm, or was quite recently, move forward.
                    return CHASE;
                }
            }
        }
        else        //we are not in evade mode, and we have a lock.
        if(((Left->Input) && (Left->Input < KILL_DISTANCE_MM)) && ((Right->Input) && (Right->Input < KILL_DISTANCE_MM)))
        {           //the object is visible to both sensors and within ~200mm; recommend full power to the motors.
            return KILL;
        }
        else
        {
            return CHASE;
        }           //the object is visible to both sensors.
    }
    if ( (Left->Input) && ( (Left->Input < CurrentMaxDistance) || (millis()-Left->AuxTime<ENEMY_DETECTED_MS_AGO) ) )
    {               //the object is visible to the front left sensor
        return CURRENTLEFT;
    }
    if ( (Right->Input) && ( (Right->Input < CurrentMaxDistance) || (millis()-Right->AuxTime<ENEMY_DETECTED_MS_AGO) ) )
    {               //the object is visible to the front right sensor
        return CURRENTRIGHT;
    }
    
    //"He's Behind You" checks - lowest priority
    if ((millis()-RearLeft->Time<ENEMY_AT_REAR_MS_AGO) || ((RearLeft->Input) && (RearLeft->Input < MAX_DISTANCE_MM)))
    {               //Rear Left sensor
        return RECENTLOCK_RIGHT;
    }
    if ((millis()-RearRight->Time<ENEMY_AT_REAR_MS_AGO) ||((RearRight->Input)&& (RearRight->Input < MAX_DISTANCE_MM)))
    {               //Rear Right sensor
        return RECENTLOCK_LEFT;
    }
    
    //Time Based Checks
    if (((millis() - Left->Time) < ENEMY_DETECTED_WHILE_AGO) || ((millis() - Right->Time) < ENEMY_DETECTED_WHILE_AGO)) //One of the sensors have had lock very recently
    {               //sensors have been high in last little while
        {
            if (Left->Time < Right->Time)
            {       //left was high most recently
                return RECENTLOCK_LEFT;
            }
            if (Right->Time <= Left->Time)
            {       //right was high most recently
                return RECENTLOCK_RIGHT;
            }
        }
    }
    return NOLOCK;
}

uint8_t FindEnemy(uint8_t Mode) //returns KILL), CHASE, 
{
    //Begin by rotating; check what time we start, rotate for n seconds.
    //If no enemy found, move straight line m seconds.
    //If still no enemy found, rotate.... and so on.

    //If enemy found but I lose lock - change rotation direction.
    //If enemy not regained and it goes historic, reset the changed dir - keep on rotating like we would if we never saw it.
    //if enemy regained, hell yes off we go to the races.
    static uint32_t TimeRotationStarted = 0;    //time we started rotation in effort to find enemy
    static uint32_t TimeStraightRunStarted=0;   //time we started going straight in effort to find enemy
    static uint8_t RotationDirection = CLOCKWISE;   //default direction of rotation
    static bool Spinny = true;      //Flag to indicate that we want to rotate
    uint8_t EnemyLocation = IsEnemyVisible(Mode); 
    uint8_t IntendedAction = MOTION_FORWARD;    //Set default movements
    uint8_t IntendedSpeed = SPEED_SEARCHING;
    bool UrgentAction = false;
    switch (EnemyLocation)
    {
        case KILL:                                      //the target is within ramming distance
        case CHASE:                                     //the target is locked but not close enough just yet
        {                                               //I have combined these two as the logic is very similar
            TimeRotationStarted = 0;                    //This is the only value of time I can be confident will never eventuate in normal usage
            Spinny = false;
                                                        //Slightly clumsily, edge & tilt avoid will be called twice in the event that I have a full lock but am tilted or on the edge
            #ifdef IMU                                  //If the Tilt detector is in use
            if(!(Mode==FULLSUMO))                       //in sumo mode, we can probably expect some tilt to occur
            {
                IntendedSpeed=AvoidTilt(IntendedSpeed);//Check for tilt, modify speed if not level
            }
            #endif
            IntendedAction = EdgeAvoid(IntendedAction);
            if (IntendedAction == MOTION_FORWARD && IntendedSpeed==SPEED_SEARCHING)     //we are neither tilted nor on the edge
            {   if((EnemyLocation==CHASE)||Mode==ATTACK)                                //in attack mode, lets go a little bit slower - there is no need for full force
                {
                    if(!(Mode==EVADE))
                    {   //we have a target lock and it is not within Full Power distance (or, in attack mode, we dont want to use full power)
                        IntendedSpeed=SPEED_CHASE;
                        Motor_Action(IntendedAction, IntendedSpeed, UrgentAction);  //move forward, quickly, allow ramped acceleration
                    }
                    else
                    {   
                        IntendedSpeed=SPEED_EVADE;              
                        Motor_Action(IntendedAction, IntendedSpeed, UrgentAction);  //move forward, slowly (The box will move at about the same speed as the robot)
                    }
                }
                else
                {
                    IntendedSpeed=SPEED_RAMMING;
                    Motor_Action(IntendedAction, IntendedSpeed, UrgentAction);      //Charge Forward, Full Noise 
                }
                return EnemyLocation;   
            }
            else
            {                           //there is an issue with either tilt or edge
                break; //Bugger!
            }
        }
        case CURRENTLEFT:
        case RECENTLOCK_LEFT:
        {
            Spinny = true;
            RotationDirection = CLOCKWISE;  //enemy to our left, so rotate clockwise
            break;
        }
        case CURRENTRIGHT:
        case RECENTLOCK_RIGHT:
        {
            Spinny = true;
            RotationDirection = COUNTERCLOCKWISE;   //enemy to our right, so rotate anticlockwise
            break;
        }
        case HESBEHINDYOU: //jaysus mary and jaahhhseph
        {
            Spinny = true;
            RotationDirection= CLOCKWISE;       //pick a direction, any direction. Ideally this would move faster, but the 20ms delay is proving an issue
            break;
        }
        case HISTORICALLOCK:    //treated the same as no lock, really
        {
            Spinny = true;
            RotationDirection = CLOCKWISE;      //
            break;
        }
        case NOLOCK:                                                    //No idea where the enemy is, so start looking
        {
            if (TimeRotationStarted == 0)                               //first time we have no lock
            {
                TimeRotationStarted = millis();                         //capture current time
                Spinny = true;                                          //start rotation
                RotationDirection = CLOCKWISE;                          //set rotation direction
            }
            if (((millis() - TimeRotationStarted) > FIRSTSPIN)&& Spinny)
            {                                                           //rotate CW for first 2 seconds
                RotationDirection=COUNTERCLOCKWISE;                     //then CCW for 2 seconds

                if ((millis() - TimeRotationStarted) > FIRSTSPIN * 2)
                {                                                       //If still no object found, move straight forwards (using whatever angle we finish rotating in)
                    Spinny = false;                                     //In this way we should obtain some form of random wander across the board
                    TimeStraightRunStarted = millis();
                }
            }
            else
            {
                RotationDirection=CLOCKWISE;
            } 
            break;
        }
        case EVADESTOP:
        {                                                               //In evade mode;
            Spinny = false;                                             //Object further than 250mm and visible to both sensors.
            IntendedAction = MOTION_STOPPED;
            break;
        }
    }

    if ((Spinny == false)&&(!(EnemyLocation==EVADESTOP)))
    {
        IntendedAction = MOTION_FORWARD;    
        if(millis()-TimeStraightRunStarted>STRAIGHTRUNTIME)//we have driven in a straight line for long enough
        {
            Spinny=true;                                    //resume rotation
            TimeRotationStarted=millis();
            TimeStraightRunStarted=0;  
            RotationDirection = CLOCKWISE;
        }
    }
    if (Spinny == true)                                     //Robot should Rotate
    {
        if (RotationDirection == CLOCKWISE)
        { 
            IntendedAction = MOTION_ROTATECW;               //set CW rotation direction
        }
        else
        {
            IntendedAction = MOTION_ROTATECCW;              //set CCW rotation direction
        }
    }
    #ifdef IMU
        if(!(Mode==FULLSUMO))                               //in sumo mode, we can probably expect some tilt to occur and wish to ignore it
            IntendedSpeed=AvoidTilt(IntendedSpeed);
    #endif
    if(IntendedSpeed==0)    //for the intended speed to now be 0, the tilt function must have come to a standstill and we are still tilted.
    //The only way out of this is for the robot to reverse?
    {
        IntendedAction=MOTION_REVERSE;              //What happens in the case where the robot is over the top of the enemy?
        IntendedSpeed=SPEED_AVOID;                  //ie, we no longer can see them so we start slowing down.
                                                    //A better option is for the robot to never tilt?
                                                    //Perhaps, once we enter KILL or CHASE state, the tilt sensor should be ignored for a period of time? Say, 30 seconds?
                                                    //TODO battle other robot, compare.
    }
    uint8_t OriginalAction = IntendedAction;    
    IntendedAction = EdgeAvoid(IntendedAction); 
    if (!(IntendedAction == OriginalAction))        //If the edge avoid function modified intended behaviour, we are too close to the edge.
    {                                               //This is an absolute priority
        IntendedSpeed = SPEED_CHASE;
        UrgentAction = true;                        //turns off the ramped acceleration; we have ONE priority, and it is not going off the board
    }
    Motor_Action(IntendedAction, IntendedSpeed, UrgentAction);  //Send action, speed, and whether or not to do a ramped acceleration to motor.
    return EnemyLocation;   //return value no longer very useful, kept for
}

