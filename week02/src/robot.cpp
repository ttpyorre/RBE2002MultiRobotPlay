#include "robot.h"
#include <hc-sr04.h>
#include "ir_codes.h"
#include <BNO055.h>

#define IR_PIN 15

#define HC_TRIG 16      //for pinging -- not a great choice since this can hamper uploading
#define HC_ECHO 17      //for reading pulse

#define MB_CTRL 2
#define MB_PULSE 35

int count = 0;
int lastHandleUpdate = 0;
BNO055 IMU;

Robot::Robot(void)
    : filter(5), irDecoder(IR_PIN)
{
    //nothing to see here; move along...
}

void Robot::init(void)
{
    chassis.init();
    irDecoder.init();
    hc_sr04.init();
    IMU.init(OPR_MODE_AMG, 35); // We are using pin 35 for the interrupt
    IMU.enableExternalCrystal(true);
}

void Robot::loop() 
{
    hc_sr04.checkPingTimer();
    
    //check the IR remote
    int16_t keyCode = irDecoder.getKeyCode();
    if(keyCode != -1)
    { 
        handleIRPress(keyCode);
    }

    /** Check the distance sensor.
     * We return true only if there is a new reading, which is passed by reference.
     * It hardly needs to be done this way, but passing by reference is a useful tool,
     * so we start with a 'lightweight' example here.
     */
    float distanceReading = 0;
    bool hasNewReading = hc_sr04.getDistance(distanceReading);
    if(hasNewReading)
    { 
        handleNewDistanceReading(distanceReading);
    }

    /**
     * Here we check if the motors/pose were recently updated.
     * If so, then call handleUpdateReady(), which does additional tasks
     */

    for (int i = 0; i < 200 ; i++){
        if(millis() - lastHandleUpdate >= 8)
        {
            //chassis.setWheelSpeeds(180,180);
            lastHandleUpdate = millis();
            handleIMUtimer();
        }
    }
 

    if(chassis.readyForUpdate) // millis() - lastUpdate >= 50
    {
        handleUpdateReady();
        //chassis.writePose();
     }
}

void Robot::handleIMUtimer(void)
{   //Serial.println("Before");
    //Serial.println(millis());

    vector<int16_t> acc = IMU.readRawAcc();
    vector<int16_t> gyro = IMU.readRawGyro();   

    Serial.println(millis());
    
    
    Serial.print("Acc x: ");    // Points through the back of the robot
    Serial.print(acc[0]);
    Serial.print('\t');
    Serial.print("y: ");        // Points to the right of the robot
    Serial.print(acc[1]);
    Serial.print('\t');
    Serial.print("z: ");        // Points downwards from the robot.
    Serial.print(acc[2]);
    Serial.println('\t'); 


    Serial.print("Gyro x: ");   // roll - CW looking from the back of the robot is positive rotation
    Serial.print(gyro[0]);
    Serial.print('\t');
    Serial.print("y: ");        // pitch - CW looking from the right wheel is positive rotation
    Serial.print(gyro[1]);
    Serial.print('\t');
    Serial.print("z: ");        // yaw - CCW looking from top of the robot is positive rotation
    Serial.print(gyro[2]);
    Serial.println("\n");
}

void Robot::handleIRPress(int16_t key)
{
    Serial.println(key);
    if(key == STOP)
    {
        chassis.stop();
        robotState = ROBOT_IDLE;

        leftSpeed = 0;
        rightSpeed = 0;

        return;
    }

    switch(robotState)
    {
        case ROBOT_IDLE:
            if(key == VOLminus) {   robotState = ROBOT_STANDOFF;}
            if(key == VOLplus)  {   robotState = ROBOT_WALL_FOLLOWING;}
            if(key == PREV)     {   robotState = DRIVE_STRAIGHT;} 
            if(key == CHminus)  {   robotState = SPIN_CCW;}

            if (key == 16){
                chassis.setDest(30,30);
                robotState = ROBOT_DRIVE_TO_POINT;
            }
            if(key == 17){
                chassis.setDest(60,0);
                robotState = ROBOT_DRIVE_TO_POINT;
            }
            if(key==18){
                chassis.setDest(30,-30);
                robotState = ROBOT_DRIVE_TO_POINT;
            }
            if(key==12){
                chassis.setDest(1,1);
                robotState = ROBOT_DRIVE_TO_POINT;
            }
            break;

        case ROBOT_STANDOFF:
            standoffController.handleKeyPress(key);
            break;
        case ROBOT_WALL_FOLLOWING:
            wallFollower.handleKeyPress(key);
            break;
        case DRIVE_STRAIGHT:
        /*
            rightSpeed = 30;
            leftSpeed = 30;
            chassis.setWheelSpeeds(leftSpeed, rightSpeed);*/
            break;
        default:
            break;
    }
}


void Robot::handleNewDistanceReading(float distanceReading)
{
    float averageDist = filter.addAndReturnMedian(distanceReading);
    // Serial.print(averageDist);
    // Serial.print('\t');

    if (robotState == ROBOT_WALL_FOLLOWING)
    {    
        wallFollower.processDistanceReading(distanceReading);
        chassis.setMotorEfforts(wallFollower.leftEffort, wallFollower.rightEffort);
    }

    if(robotState == ROBOT_STANDOFF)
    {
        standoffController.processDistanceReading(averageDist);
        chassis.setMotorEfforts(standoffController.leftEffort, standoffController.rightEffort);
    }   

    if(robotState == ROBOT_DRIVE) //if we're driving and we see an object that's too close, stop
    {
        //TODO: check if there is an object too close and emergency stop
        if(distanceReading <= 20)
        {
            chassis.setMotorEfforts(0, 0);
        }
    }
}

/**
 * This gets called whenever the drive motors and pose get aated.
 */
void Robot::handleUpdateReady(void)
{
    chassis.readyForUpdate = false;

    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        //TODO: calculate motor inputs to give desired motion

        //TODO: check if we've reached destination
        chassis.driveToPoint();
        chassis.setWheelSpeeds(chassis.Vleft,chassis.Vright);

        if (chassis.checkDestination()){
            chassis.stop();

            robotState = ROBOT_IDLE;
        }


    }
    if(robotState == DRIVE_STRAIGHT)
    {
        rightSpeed = 180;
        leftSpeed = 180;
        chassis.setWheelSpeeds(rightSpeed, leftSpeed);

    } 
    else if (robotState == SPIN_CCW)
    {
        rightSpeed = 180;
        leftSpeed = -180;
        chassis.setWheelSpeeds(leftSpeed, rightSpeed);
    }
}
   

