#pragma once

#include <RBE-200n-Lib.h>
#include <ESP32Servo.h>
#include <MotorEncoded.h>
#include "pose.h"
#include <BNO055.h>

/**
 * Chassis class
 */

#define WHEEL_TRACK	13.95 //cm
#define WHEEL_RADIUS 3.455 //cm // 3.46
#define DELTA_T 0.05 //Seconds

class Chassis
{
protected:
	/**
	 * Loop rate for this motor. The main timer loop is 1ms, so this value is in ms.
	 */
	uint32_t controlIntervalMS = DRIVE_MOTOR_CTRL_INT_MS;

	/**
	 * An internal counter that counts iterations of the main motor loop; it is compared to
	 * controlIntervalMS so that the velocity PID runs on longer intervals.
	 */
	uint32_t velocityLoopCounter = 0;

    MotorEncoded leftMotor;
    MotorEncoded rightMotor;

    bool timerAllocated = false;
    void allocateTimer(void); 
    void motorHandler(void);
    friend void onMotorTimer(void* param);

    float k = 0.875, angleOld = 0;

    double biasOld = 0, e = 0;
    double thetaGyro = 0, thetaAcc = 0;
    double estimatePitch;

    Pose currPose;
    Pose destPose;
    PIDController Heading;
    PIDController Distance;
    PIDController romioHitTybot;
    bool isCorrecting=false;
public:
    bool readyForUpdate = false;
    float Vleft,Vright;
    BNO055 IMU;
    
public:
    Chassis(void);
    void init(bool IMUon);
    void loop(void);
    void hitTybot(int error);
    double getTotalMotorSpeed();
    void stop(void) {setMotorEfforts(0,0);}
    double estimatedPitchAngle(void);
    double updateGyroBias(void);
    void setWheelSpeeds(float left, float right);
    void setMotorEfforts(float left, float right);
    float clampEffort(float effort, float max,float min);
    void setTwist(float u, float omega);    //implementation left to the student
    void updatePose(float leftDelta, float rightDelta);          //implementation left to the student
    void writePose(void);

    void driveToPoint(bool isReversing);                //implementation left to the student
    bool checkDestination(void);
    void setDest(int, int);
    Pose getPose();
};
