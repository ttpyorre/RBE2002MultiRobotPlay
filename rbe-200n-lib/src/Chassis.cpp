#include <Chassis.h>
#include <cmath>

#define DEGREETORADIAN 0.0174532925199
#define IMU_UPDATE_TIME 0.01 //in s, so 10ms

/**
 * Chassis class
 */

Chassis::Chassis(void) :
    leftMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB),
    rightMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB),
	Heading(15,1.5,7,7), Distance (10,0.05,0.1,0),romioHitTybot(1,0,0,0)

{
    //TODO: Set gains for speed control
	//Either click into the MotorEncoded class and change the initializer in the 
	//constructor, or manually set the gains with the setter method: setPIDgains(...)

	// Done :)
}

void Chassis::init(bool IMUon)
{
	allocateTimer(); // used by the DC Motors

	leftMotor.attach();
    rightMotor.attach();

	//Initialize the position at hthe beginning
	//Only initialze once in the robot.cpp init
	currPose.x = 0;
	currPose.y = 0;
	currPose.theta = 0 ;
	if(IMUon){
	IMU.init(OPR_MODE_AMG, 35); // We are using pin 35 for the interrupt
    IMU.enableExternalCrystal(true);
	}
	stop();
}

void Chassis::setWheelSpeeds(float left, float right) 
{
    leftMotor.setTargetDegreesPerSecond(left);
    rightMotor.setTargetDegreesPerSecond(right);
}

void Chassis::setMotorEfforts(float left, float right) 
{
    leftMotor.setEffortPercent(left);
    rightMotor.setEffortPercent(right);
}

//speed in cm/sec; omega in rad/sec
void Chassis::setTwist(float speed, float omega)
{
	//TODO: calculate desired motor speeds for given motion 
	Vleft = speed + (omega * WHEEL_TRACK) / 2;
    Vright = speed - (omega * WHEEL_TRACK) / 2;
	// Serial.print(speed);
	// Serial.print('\t');
	// Serial.print(omega);
	// Serial.print('\t');
	// Serial.print(Vleft);
	// Serial.print('\t');
	// Serial.println(Vright);

}

double Chassis::getTotalMotorSpeed()
{
	return leftMotor.getDegreesPerSecond()+rightMotor.getDegreesPerSecond();
}

float Chassis::clampEffort(float effort, float min, float max)
{
	if (effort > max)
	{
		effort = max;
	}
	else if (effort < min)
	{
		effort = min;
	}
	return effort;
}

void Chassis::hitTybot(int error)
{
	double stabbingEffort = romioHitTybot.computeEffort(error);
	setWheelSpeeds(700 + stabbingEffort, 700 + stabbingEffort);
}

void Chassis::updatePose(float leftDelta, float rightDelta) //parameters in degrees of wheel rotation
{


	//TODO : Convert left and right Delta to cm
	leftDelta *= WHEEL_RADIUS * DEGREETORADIAN;
	rightDelta *= WHEEL_RADIUS * DEGREETORADIAN;

	float distance = (leftDelta  + rightDelta) / 2;
	float deltaTheta = (rightDelta  - leftDelta) / WHEEL_TRACK;
	float instantaneousCenterRadius, deltaX, deltaY;

	if (leftDelta == rightDelta) 
	{
		deltaTheta = 0;

		deltaX = distance * float(cos(currPose.theta));
	 	deltaY = distance * float(sin(currPose.theta));
	}
	else 
	{
		instantaneousCenterRadius = (WHEEL_TRACK / 2) * ((leftDelta + rightDelta) / (rightDelta - leftDelta));
	
	 	deltaX = instantaneousCenterRadius * float((sin(currPose.theta + deltaTheta) - sin(currPose.theta)));
	 	deltaY = instantaneousCenterRadius * float((-cos(currPose.theta + deltaTheta) + cos(currPose.theta)));
	}

	currPose.x += deltaX;
	currPose.y += deltaY;
	currPose.theta += deltaTheta;

}

Pose Chassis::getPose()
{
	return currPose;
}

void Chassis::writePose(void)
{
	
	Serial.print(currPose.x);
	Serial.print('\t');
	Serial.print(currPose.y);
	Serial.print('\t');
	Serial.print(currPose.theta);
	Serial.println('\t');
}

void Chassis::setDest(int x, int y)
{
	destPose.x = x;
	destPose.y = y;
	destPose.theta = atan2(y, x);
}

void Chassis::driveToPoint(bool isReversing) 
{	
	float errorTheta = 0;
	//TODO: you'll need to add PID control here, both for distance and heading!
	float errorDistance = sqrt(pow(destPose.x - currPose.x, 2) + pow(destPose.y - currPose.y, 2));

	if(destPose.x - currPose.x >= 0)
	{
		errorTheta = -atan2((destPose.y - currPose.y), (destPose.x - currPose.x)) + currPose.theta;
	}
	else
	{
	 	//Serial.print("reversing");
	 	errorTheta = -atan2((-destPose.y + currPose.y), (-destPose.x + currPose.x)) + (currPose.theta - PI);
	}
	if(isReversing){
		errorTheta=errorTheta+PI;
	}
    float omega = Heading.ComputeEffort(errorTheta);
    float speed = 0;
	//Serial.println(errorTheta);
	isCorrecting=false;
	if(isCorrecting == true)
	{
		speed = 0;
	} 
	else
	{
		speed = Distance.ComputeEffort(errorDistance);
		if(isReversing){
			speed=speed*-1;
		}
	}

    setTwist(speed, omega);

	if(abs(errorTheta) > 0.5)
	{
		isCorrecting = true;
	} 
	else if(abs(errorTheta) < 0.1)
	{
		isCorrecting = false;
	}
	
	//Declare one speed
	//Find R 
	//Determine the other speed by PID control (??)
}

bool Chassis::checkDestination(void) 
{
	//TODO: Check if you've reached destination
	if(abs(currPose.x - destPose.x) < 4)  // && (abs(currPose.y - destPose.y) < 4))
	{
		return true;
	}
	
	return false;
}

void Chassis::motorHandler(void)
{
    if(!timerAllocated) allocateTimer();

	if(++velocityLoopCounter == controlIntervalMS) 
	{
		velocityLoopCounter = 0;

		leftMotor.process();
		rightMotor.process();

		//here's where you'll update the pose in Lab 3, née 2
		updatePose(leftMotor.getDelta(), rightMotor.getDelta());

		readyForUpdate = true;
	}
}

static TaskHandle_t complexHandlerTask;

void onMotorTimer(void* param)
{
	ESP_LOGI(TAG, "Starting the PID loop thread");
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = 1; 	// sets up 1ms timer
	while(true)
	{
		vTaskDelayUntil(&xLastWakeTime, xInterval);
		Chassis* pChassis = (Chassis*) param;
		if(pChassis) pChassis->motorHandler();
		else Serial.println("NULL pointer in onMotorTimer()!!");
	}
	ESP_LOGE(TAG, "ERROR PID thread died!");
}

void Chassis::allocateTimer(void)
{
	if (!timerAllocated)
	{
		xTaskCreatePinnedToCore(onMotorTimer, "PID loop Thread", 8192, this, 1, &complexHandlerTask, 0);
	}

	timerAllocated = true;
}

double Chassis::estimatedPitchAngle(void)
{
	//Serial.println("Starting estimatedPitch");
    vector<double> acc = IMU.calculateAcceleration();
	//Serial.println("EP CA");
    vector<double> gyro = IMU.gyroToAngularVec();
	//Serial.println("EP GAV");

    thetaAcc = IMU.getPitchAcc() * DEGREETORADIAN;
    thetaGyro = angleOld + ((gyro[1] - updateGyroBias()) * IMU_UPDATE_TIME);
	//Serial.println("EP Theta math");
    estimatePitch = (k * thetaGyro) + ((1 - k) * thetaAcc);
    angleOld = estimatePitch;
	//Serial.println("finished estimatedPitch");

    return estimatePitch;

}

double Chassis::updateGyroBias(void)
{
	//16 here is the sensitivity of the gyroscope, 0.01 is the change in seconds here
    double bias = biasOld + (e * (16/IMU_UPDATE_TIME) * (thetaAcc - thetaGyro));
    biasOld = bias;
    return bias;
}
