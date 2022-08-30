#include "robot.h"
#include <hc-sr04.h>
#include "ir_codes.h"
#include <BNO055.h>
#include <mqtt.h>

#define IR_PIN 15

#define MB_CTRL 2
#define MB_PULSE 35

#define LEDPININIT 33
#define EMITTERPIN 18

//We define the maximum and minimum for clampEffort
#define MAX 300
#define MIN -300

Robot::Robot(void)
    : filter(5), irDecoder(IR_PIN)
{
    //nothing to see here; move along...
}

void Robot::init(void)
{
    if(robotMQTTState == TYBOT_AND_JULIBOT)
    {
        //Initializes the BNO055
        chassis.init(true);
    }
    else
    {
        chassis.init(false);
    } 
    
    irDecoder.init();
    irFinder.begin();
    hc_sr04.init();
    
    pinMode(LEDPININIT, OUTPUT);
    pinMode(EMITTERPIN, OUTPUT);
    
}

void Robot::loop() 
{           
    client.loop();
    hc_sr04.checkPingTimer();
    currPose = chassis.getPose();

    //Handles the MQTT update every 1 second. 
    if(millis() - lastHandleMQTT >= 1000)
    {
        lastHandleMQTT = millis();
        handleMQTTupdate();
    }

    //Handles updating the IMU
    if(millis() - lastHandleUpdateIMU >= 10 && robotMQTTState == TYBOT_AND_JULIBOT)
    {
        lastHandleUpdateIMU = millis();
        handleIMUtimer();
    }

    //Handles which scene we start with for all Romi's 
    int16_t keyCode = irDecoder.getKeyCode();
    if (keyCode == NUM_1)
    {
        startProgram = true;
        robotState = TYBOT_INIT;
    }
    else if (keyCode == NUM_2)
    {
        startProgram = true;
        robotState = JULIBOT_INIT;
    }
    else if (keyCode == NUM_3)
    {
        startProgram = true;
        robotState = CURTAIN_CALL;
    }

    //Runs the whole program, and takes a keycode so we can have an emergency stop at anytime.
    handleIRPress(keyCode);
}

void Robot::handleIRPress(int16_t key)
{
    //We can stop the program with this anytime.
    if(key == STOP)
    {
        chassis.stop();
        robotState = ROBOT_IDLE;

        leftSpeed = 0;
        rightSpeed = 0;
    }

    switch (robotState)
    {
    //Initial state for all Romi's
    case ROBOT_IDLE:
        chassis.stop();
        if(millis() - lastHandleLED > 200)
        {
            lastHandleLED = millis();
            if(LEDOn)
            {
                LEDOn = false;
                digitalWrite(LEDPININIT, HIGH);
            }
            else
            {
                LEDOn = true;
                digitalWrite(LEDPININIT, LOW);
            }
        }
        break;

        //Romio Fight cases: --------------------------------------------------------------------------------------
    case ROMIO_INIT_FIGHT:
        digitalWrite(33, LOW);
        if(!romioOnStage)
        {    
            romioOnStage = getInTheStage(75);
        } 
        else
        {
            chassis.stop();
            romioOnStage = true;
        }
        break;

    case ROMIO_FIGHT_STAGE:
        //Find Tybot
        //Turning until we see Tybot
        if (haveWeSeenTybot == false)
        {
            chassis.setWheelSpeeds(300, -300);
        }

        irFinder.requestPosition();

        if (irFinder.available())
        {
            for (int i = 0; i < 4; i++)
            {
                point = irFinder.ReadPoint(i);

                if (point.y < 1000)
                {
                    haveWeSeenTybot = true;
                    tybotError = 300 - point.y;
                    
                    chassis.hitTybot(tybotError);
                }
            }
            Serial.print('\n');
        }
        else
        {
            Serial.println("IR camera not available!");
        }
        //Puttin startTime for next program.
        startTime = millis();
        break;
    case ROMIO_RUN:
        //Leaves back to the original position
        chassis.setDest(0, 0);
        chassis.driveToPoint(true);
        chassis.writePose();

        chassis.setWheelSpeeds(-chassis.clampEffort(chassis.Vleft/4, MAX, MIN), -chassis.clampEffort(chassis.Vright/4, MAX, MIN));
        
        //Puts motor efforts to 0, when we have returned back to the original position. Or it has been 4s since that time.
        if (chassis.checkDestination() || millis() - startTime > 4000)
        {
            chassis.stop();
            robotState = ROBOT_IDLE;
        }
        break;

        //Tybot Cases: --------------------------------------------------------------------------------------
    case TYBOT_INIT:
        //Tybot going on stage
        digitalWrite(LEDPININIT, LOW);
        isOnStage = getInTheStage(75);
        if (isOnStage)
            {robotState = TYBOT_CIRCLING;}

        break;
    case TYBOT_CIRCLING:
        mercutibotDistance = 0;
        hasNewReading = hc_sr04.getDistance(mercutibotDistance);
        if (hasNewReading)
            {standoffController.processDistanceReading(mercutibotDistance);}
        
        //Setting the wheel efforts
        chassis.setMotorEfforts(standoffController.leftEffort, standoffController.rightEffort);
        break;
    case TYBOT_STAB:
        //Goes in for the stab.
        chassis.setMotorEfforts(60, 60);

        if (detectCollision())
        {
            chassis.stop();
            AccOld = {0, 0, 0};
            robotState = TYBOT_STAB_COMPLETE;
        }
        break;
    case TYBOT_STAB_COMPLETE:
        //Tybot done stabbing, then waits for Romio.
        digitalWrite(EMITTERPIN, LOW);
        tone(EMITTERPIN, 38000);

        if (ifIamStabbed())
            {robotState = TYBOT_DEAD;}
        break;
    case TYBOT_DEAD:
        //Sets red LED to indicate dying.
        digitalWrite(33, HIGH);
        chassis.stop();
        break;
    //Mercutibot Cases: --------------------------------------------------------------------------------------
    case MERCUTIBOT_INIT:
        digitalWrite(LEDPININIT, LOW);
        isOnStage = getInTheStage(75);
        if (isOnStage)
            {robotState = MERCUTIBOT_CIRCLING;}
        break;
    case MERCUTIBOT_CIRCLING:
        //Go back and forth for 2 times
        //Stop and send a signal to Tybot "Okay I'm ready, stab me"
        if (circlingCount < 5)
        {
            //Goes forwards
            if (!doneForward)
            {
                chassis.setDest(105, 0);
                chassis.driveToPoint(false);
                chassis.writePose();
                chassis.setWheelSpeeds(-chassis.clampEffort(chassis.Vleft/4, MAX, MIN), -chassis.clampEffort(chassis.Vright/4, MAX, MIN));
            }
            //Goes backwards
            else if (doneForward)
            {
                chassis.setDest(75, 0);
                chassis.driveToPoint(true);
                chassis.writePose();
                chassis.setWheelSpeeds(-chassis.clampEffort(chassis.Vleft/4, MAX, MIN), -chassis.clampEffort(chassis.Vright/4, MAX, MIN));
            }

            if (chassis.checkDestination())
            {
                chassis.stop();
                if (chassis.getTotalMotorSpeed() < 100)
                {
                    doneForward = !doneForward;
                    circlingCount++;
                }
            }
        }
        else
        {
            chassis.stop();
            MercutibotIsReadyForStab = true;
            robotState = MERCUTIBOT_WAITING_FOR_STAB;
        }
        break;
    case MERCUTIBOT_WAITING_FOR_STAB:
        //Wait till Tybot stabs him
        break;
    case MERCUTIBOT_DEAD:
        //Bleeds
        digitalWrite(33, HIGH);
        break;

        //Romio Balcony Cases: --------------------------------------------------------------------------------------
    case ROMIO_BALCONY_INIT:
        digitalWrite(33, LOW);
        break;
    case ROMIO_BALCONY_ENTER:
        //Going to the stage
        chassis.setDest(100, 0);

        if(romio.FindAprilTags() < 1)
        {
            chassis.driveToPoint(false);
            chassis.setWheelSpeeds(-chassis.clampEffort(chassis.Vleft / 4, MAX, MIN), -chassis.clampEffort(chassis.Vright / 4, MAX, MIN));
        }
        else
        {
            robotState = ROMIO_BALCONY_FOLLOW;
        }
        break;
    case ROMIO_BALCONY_FOLLOW:
        //Following Julibot
        romio.FindAprilTags();
        romio.tagStandoff();

        chassis.setMotorEfforts(romio.leftEffort, romio.rightEffort);
        break;
    case ROMIO_ENDING:
        chassis.stop();
        break;
        //Julibot Cases: --------------------------------------------------------------------------------------
    case JULIBOT_INIT:
        //Initializes for MQTT, MQTT can take a second to publish into the MQTT.
        if (!initialize)
        {
            startTime = millis();
            initialize = true;
        }
        
        digitalWrite(33, LOW);
        if (millis() - startTime > 1000)
        {
            initialize = false;
            robotState = JULIBOT_CLIMB;
        }
        break;
    case JULIBOT_CLIMB:
        //Julibot climbs the ramp, using wall following at the beginning.
        julibotDistance = 0;
        hasNewReading = hc_sr04.getDistance(julibotDistance);
        if (hasNewReading)
        {
            wallFollower.processDistanceReading(julibotDistance);
        }
        
        chassis.setMotorEfforts(wallFollower.leftEffort, wallFollower.rightEffort);
        //Hysteresis band for climbing the ramp.
        if (pitchAngle > 0.15)
        {
            isClimbing = true;
        }
        if (isClimbing == true && pitchAngle < 0.02)
        {
            chassis.stop();
            isClimbing = false;
            robotState = JULIBOT_BROADCAST;
        }
        break;
    case JULIBOT_BROADCAST:
        //wait for ROMIO to go into the stage and detect Julibot
        break;
    case JULIBOT_FOLLOW:
        //Julibot gets hit by Fribot, and is on the ground after having climbed the ramp to end the scene.
        if ((pitchAngle < 0.02) && (detectCollision() == true) && (isClimbing == true)) 
        {
            robotState = JULIBOT_ENDING;
        }
        else if(pitchAngle > 0.15)
        {
            isClimbing = true;
        }

        //The wall following happens.
        julibotDistance = 0;
        hasNewReading = hc_sr04.getDistance(julibotDistance);
        if (hasNewReading)
        {
            wallFollower.processDistanceReading(julibotDistance);
        } 
        chassis.setMotorEfforts(-wallFollower.leftEffort, -wallFollower.rightEffort);
        break;
    case JULIBOT_ENDING:
        chassis.stop();
        break;
        //Fribot Cases: --------------------------------------------------------------------------------------
    case FRIBOT_INIT:
        //Initializing Fribot.
         break;
    case FRIBOT_ENTER:
        chassis.setDest(40, 0);
        chassis.driveToPoint(false);
        chassis.setWheelSpeeds(-chassis.clampEffort(chassis.Vleft / 4, MAX, MIN), -chassis.clampEffort(chassis.Vright / 4, MAX, MIN));

        if(chassis.checkDestination() == true)
        {
            chassis.stop();
            robotState = FRIBOT_ENDING;
        }
        break;
    case FRIBOT_ENDING:
        //Celebrating :)
        break;
        //End of Romi cases: --------------------------------------------------------------------------------------
    case CURTAIN_CALL:
        //When we have arrived at a destination, we switch the state.
        if (isOnStage)
            {robotState = CURTAIN_CALL_PERFORMANCE;}
        //Dependent on which robot it is, it goes to that destination.
        else if(robotMQTTState == ROMIO)
            {isOnStage = getInTheStage(25);}
        else if(robotMQTTState == TYBOT_AND_JULIBOT)
            {isOnStage = getInTheStage(50);}
        else if(robotMQTTState == MERCUTIBOT_AND_FRIBOT)
            {isOnStage = getInTheStage(75);}
        break;
    case CURTAIN_CALL_PERFORMANCE:
        //Checks that the Romi has turned aroun ~90degrees
        currPose = chassis.getPose();
        if(currPose.theta * 180.0/PI > 89 && millis() - lastHandleLED > 150)
        {
            chassis.stop();
            lastHandleLED = millis();
            startBack++;
            
            //Performance happens
            if(startBack > 15)
            {
                digitalWrite(33, HIGH);
                robotState = CURTAIN_CALL_BACK;
                isOnStage = false;
            }
            else if(LEDOn)
            {
                LEDOn = false;
                digitalWrite(33, HIGH);
            }
            else
            {
                LEDOn = true;
                digitalWrite(33, LOW);
            }
        }
        else
        {
            chassis.setMotorEfforts(0, 10);
        }
        break;
    case CURTAIN_CALL_BACK:
        currPose = chassis.getPose();

        //Turns around to turn to go back off-stage. Then goes back out.
        if(currPose.theta * 180.0/PI > 179)
        {  
            isOnStage = getInTheStage(0);
            if (isOnStage)
            {
                chassis.stop();
                robotState = DOING_NOTHING;
            }
        }
        else
        {
            chassis.setMotorEfforts(0, 10);
        }
        break;
    case DOING_NOTHING:
        //Is off-stage, and goes off.
        digitalWrite(33, LOW);
        break;
    default:
        break;
    }
}

//Getting in the stage code for all Romi's
bool Robot::getInTheStage(int destination)
{   
    Serial.print(-chassis.clampEffort(chassis.Vleft/4, MAX, MIN));
    Serial.print('\t');
    Serial.println(-chassis.clampEffort(chassis.Vright/4, MAX, MIN));

    chassis.setDest(destination, 0);
    chassis.driveToPoint(false);

    chassis.setWheelSpeeds(-chassis.clampEffort(chassis.Vleft/4, MAX, MIN), -chassis.clampEffort(chassis.Vright/4, MAX, MIN));
    chassis.writePose();

    if (chassis.checkDestination())
    {
        chassis.stop();
        return true;
    }
    return false;
}

//This gets called whenever the drive motors and pose get called.
void Robot::handleUpdateReady(void)
{
    chassis.readyForUpdate = false;
}

//Handles the IMU
void Robot::handleIMUtimer(void)
{
    pitchAngle = chassis.estimatedPitchAngle();
}

//---------------------------------Julibot and Tybot commands-----------------------------------
//Julibot doing the wall following
void Robot::handleNewDistanceReading(float distanceReading)
{
    float averageDist = filter.addAndReturnMedian(distanceReading);
    //Julibot does it forwards
    if (robotState == JULIBOT_CLIMB)
    {    
        wallFollower.processDistanceReading(averageDist);
        chassis.setMotorEfforts(wallFollower.leftEffort, wallFollower.rightEffort);
    }

    //Julibot does it backwards.
    if (robotState == JULIBOT_FOLLOW)
    {    
        wallFollower.processDistanceReading(averageDist);
        chassis.setMotorEfforts(-wallFollower.leftEffort, -wallFollower.rightEffort);
    }
    
}

//Detect collision for Tybot, when he stabs
bool Robot::detectCollision()
{
    bool collision = false;
    vector<double> acc = chassis.IMU.calculateAcceleration();

    if (acc[0] - AccOld[0] < -6.25)
    {
        collision = true;
    }
    AccOld = acc;

    return collision;
}

//Detect collision for Tybot, when he gets stabbed
bool Robot::ifIamStabbed()
{
    bool stabbed = false;
    vector<double> acc = chassis.IMU.calculateAcceleration();

    if (abs(acc[1] - AccOld[1]) > 3)
    {
        stabbed = true;
    }
    AccOld = acc;

    return stabbed;
}

//---------------------------------MQTT Functions-----------------------------------------------
//Handles the updates for the MQTT
void Robot::handleMQTTupdate(void) 
{ 
    currPose = chassis.getPose(); 

    if (!client.connected())  
    {         
        reconnect(); 
    }
    
    switch(robotMQTTState)
    {
        case ROMIO:
            client.publish("team11/Romio/State", castEnumToString().c_str());

            client.publish("team11/Romio/pose/x", String(currPose.x).c_str()); 
            client.publish("team11/Romio/pose/y", String(currPose.y).c_str()); 
            client.publish("team11/Romio/pose/theta", String(currPose.theta * 180.0/PI).c_str());
            break;

        case MERCUTIBOT_AND_FRIBOT:
            client.publish("team11/Mercutibot&Fribot/State", castEnumToString().c_str());

            client.publish("team11/Mercutibot&Fribot/pose/x", String(currPose.x).c_str()); 
            client.publish("team11/Mercutibot&Fribot/pose/y", String(currPose.y).c_str()); 
            client.publish("team11/Mercutibot&Fribot/pose/theta", String(currPose.theta * 180.0/PI).c_str());
            break;

        case TYBOT_AND_JULIBOT:
            client.publish("team11/Tybot&Julibot/State", castEnumToString().c_str()); 

            client.publish("team11/Tybot&Julibot/pose/x", String(currPose.x).c_str()); 
            client.publish("team11/Tybot&Julibot/pose/y", String(currPose.y).c_str()); 
            client.publish("team11/Tybot&Julibot/pose/theta", String(currPose.theta * 180.0/PI).c_str());
            client.publish("team11/Tybot&Julibot/pose/pitch", String(pitchAngle).c_str());
            break;
    }
}

//Makes the current Romi state into a string that we can call from MQTT.
String Robot::castEnumToString()
{
    String robotStateString;
    switch (robotState)
    {
        //Robot Idle
        case (ROBOT_IDLE):
            robotStateString = "O";
            break;

        //Romio Fight States work with qwerty as string
        case (ROMIO_INIT_FIGHT):
            robotStateString = "q";
            break;
        case (ROMIO_FIGHT_STAGE):
            robotStateString = "e";
            break;
        case (ROMIO_RUN):
            robotStateString = "t";
            break;

        //Tybot States works with asdfg as string
        case (TYBOT_INIT):
            robotStateString = "a";
            break;
        case (TYBOT_STAB):
            robotStateString = "s";
            break;
        case (TYBOT_STAB_COMPLETE):
            robotStateString = "d";
            break;
        case (TYBOT_CIRCLING):
            robotStateString = "f";
            break;
        case (TYBOT_DEAD):
            robotStateString = "g";
            break;

        //Mercutibot States work with zxcvb
        case (MERCUTIBOT_INIT):
            robotStateString = "z";
            break;
        case (MERCUTIBOT_CIRCLING):
            robotStateString = "x";
            break;
        case (MERCUTIBOT_WAITING_FOR_STAB):
            robotStateString = "c";
            break;
        case (MERCUTIBOT_DEAD):
            robotStateString = "v";
            break;
        //Balcony Scene States: ---------------------------
        //Romio Balcony States work with QWERTY
        case (ROMIO_BALCONY_INIT):
            robotStateString = "Q";
            break;
        case (ROMIO_BALCONY_ENTER):
            robotStateString = "W";
            break;
        case (ROMIO_BALCONY_FOLLOW):
            robotStateString = "E";
            break;
        case (ROMIO_ENDING):
            robotStateString = "R";
            break;

        //Julibot States work with ASDFG
        case (JULIBOT_INIT):
            robotStateString = "A";
            break;
        case (JULIBOT_CLIMB):
            robotStateString = "S";
            break;
        case (JULIBOT_BROADCAST):
            robotStateString = "D";
            break;
        case (JULIBOT_FOLLOW):
            robotStateString = "F";
            break;
        case (JULIBOT_ENDING):
            robotStateString = "G";
            break;

        //Fribot States work with ZXCVB
        case (FRIBOT_INIT):
            robotStateString = "Z";
            break;
        case (FRIBOT_ENDING):
            robotStateString = "X";
            break;

        //Cases for Curtain Call:
        case (CURTAIN_CALL):
            robotStateString = "J";
            break;
        case (CURTAIN_CALL_BACK):
            robotStateString = "K";
            break;
        case (CURTAIN_CALL_PERFORMANCE):
            robotStateString = "L";
            break;

        default:
            break;
    }
    return robotStateString;
}
