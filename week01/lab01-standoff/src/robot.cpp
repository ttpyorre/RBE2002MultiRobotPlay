#include "robot.h"
#include "ir_codes.h"
#include <hc-sr04.h>

Robot::Robot(void) : irDecoder(IR_PIN) {}

void Robot::init(void)
{
    chassis.init();
    irDecoder.init();//IR_PIN);
    hc_sr04.init();
}

void Robot::loop() 
{
    hc_sr04.checkPingTimer();

    //check the IR remote
    int16_t keyCode = irDecoder.getKeyCode();
    if(keyCode != -1) 
    {
        handleIRPress(keyCode);
        Serial.print(String(keyCode));
        Serial.print('\n'); 
    }

    
    //check the distance sensor
    float distanceReading = 0;
    bool hasNewReading = hc_sr04.getDistance(distanceReading);
    if(hasNewReading) 
    {    
        handleNewDistanceReading(distanceReading);
    }
}

void Robot::handleIRPress(int16_t key)
{
    Serial.println(key);
    if(key == STOP)
    {
        chassis.stop();
        robotState = ROBOT_IDLE;
        return;
    }

    switch(robotState)
    {
        case ROBOT_IDLE:
            if(key == BACK)
            {
                robotState = ROBOT_STANDOFF;
            }
            if(key == PREV)
            {
                robotState = ROBOT_WALL_FOLLOWING;
            }
            break;
        case ROBOT_STANDOFF:
            standoffController.handleKeyPress(key);
            break;
        case ROBOT_WALL_FOLLOWING:
            wallFollower.handleKeyPress(key);
            break;
        default:
            break;
    }
}

void Robot::handleNewDistanceReading(float distanceReading)
{
    if(robotState == ROBOT_STANDOFF)
    {
        standoffController.processDistanceReading(distanceReading);
        chassis.setMotorEfforts(standoffController.leftEffort, standoffController.rightEffort);
    }   

    if (robotState == ROBOT_WALL_FOLLOWING)
    {    
        wallFollower.processDistanceReading(distanceReading);
        chassis.setMotorEfforts(wallFollower.leftEffort, wallFollower.rightEffort);
    }
}