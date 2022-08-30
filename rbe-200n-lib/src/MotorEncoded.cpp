/*
 * MotorEncoded.cpp
 *
 * Author: Greg Lewin, adapted from original work by hephaestus
 */
#include <MotorEncoded.h>

/**
 * Constructor for an encoded motor that uses pwm and direction pins + quadrature encoder.
 * 
 * todo: create option for paired direction pins (eg, like the MC33926)
 */
MotorEncoded::MotorEncoded(int pwmPin, int dirPin, int encAPin, int encBPin)
	: MotorBase(pwmPin, dirPin), speedController(0.005, 0.001, 0, 0) //TODO: set gains
{
	MotorEncAPin = encAPin;
	MotorEncBPin = encBPin;

	degreesPerTick = 360.0 / (encoderCountsPerMotorRev * gearSpeedRatio);
}

/**
 * Destructor pauses the encoder before removing object.
 */
MotorEncoded::~MotorEncoded()
{
	encoder.pauseCount();
}

/**
 * Sets up the encoders and calls the base class attach (which registers the motor in the list)
 */
bool MotorEncoded::attach(void)
{
	if(MotorBase::attach())
	{
		if(!encodersEnabled)
		{
			encodersEnabled = true;
			ESP32Encoder::useInternalWeakPullResistors = UP;
			encoder.attachFullQuad(MotorEncAPin, MotorEncBPin);
		}

		return true;
	}

	// motor is not attached
	return false;
}	

/**
 * Sets the target speed in degrees/second.
 * 
 * If the motor is not currently speed controlled, it resets the encoder counts; otherwise
 * there can be a jump in the motor since the error can be artificially large.
 * 
 * @param dps the new speed in degrees per second
 */
void MotorEncoded::setTargetDegreesPerSecond(float dps)
{
	attach();

	if(ctrlMode != CTRL_SPEED)
	{
		Serial.println("Resetting encoder");
		resetEncoder(); //avoids jumps when engaging control algorithm
	}

	// convert dps to ticks per control interval to avoid all the maths each time through the loop
	targetTicksPerInterval = dps * (controlIntervalMS / 1000.) / degreesPerTick;

	ctrlMode = CTRL_SPEED;
}

/**
 * This method is called for each motor when it is time to perform control.
 * 
 * Currently, it only implements speed control.
 * 
 * todo: implement position control; implement speed control through position control (red queen)
 *
 */
void MotorEncoded::process()
{
	// ensure that the motor is attached
	attach();

	//update the encoder regardless of whether or not we're going to perform control
	//this prevents jumps when changing control algorithms
	currEncoder = encoder.getCount();
	if(isReversed)
	{
		currEncoder *= -1;
	} 

	currTicksPerInterval = currEncoder - prevEncoder;
	prevEncoder = currEncoder;

	if(ctrlMode == CTRL_SPEED)
	{

		float error = targetTicksPerInterval - currTicksPerInterval;
		float effort = speedController.computeEffort(error);

		MotorBase::setEffort(effort);
	}

	MotorBase::process();
}

/**
 * getDegreesPerSecond
 *
 * This function returns the current speed of the motor
 *
 * @return the speed of the motor in degrees per second
 */
float MotorEncoded::getDegreesPerSecond()
{
	float ticksPerInterval = currTicksPerInterval;

	return ticksPerInterval * degreesPerTick * (1000.0 / controlIntervalMS);
}

float MotorEncoded::getDelta()
{
	float ticksPerInterval = currTicksPerInterval;

	return ticksPerInterval * degreesPerTick;
}

/**
 * getCurrentDegrees
 *
 * This function returns the current position in degrees (since last encoder reset)
 * @return count
 */
float MotorEncoded::getCurrentDegrees()
{
	float tmp = currEncoder;
	return tmp * degreesPerTick;
}

void MotorEncoded::setPIDgains(float kp, float ki, float kd, float cap)
{
	speedController.setKp(kp);
	speedController.setKi(ki);
	speedController.setKd(kd);
	speedController.setCap(cap);
}
