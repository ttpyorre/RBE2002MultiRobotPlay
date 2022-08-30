/*
 * MotorBase.cpp
 *
 * Author: Greg Lewin, adapted from original work by hephaestus
 */

#include <MotorEncoded.h>

MotorBase::MotorBase(int pwmPin, int dirPin)
{
	MotorPWMPin = pwmPin;
	directionPin = dirPin;
}

MotorBase::~MotorBase()
{
	pwm.detachPin(MotorPWMPin);
}

/**
 * process a basic motor.
 * sets the actual pwm level.
 */
void MotorBase::process(void)
{
	setEffortLocal(currentEffort);
}

bool MotorBase::attach(void)
{
	if(!isAttached)
	{
		pwm.attachPin(MotorPWMPin, 20000, 12);
		pinMode(directionPin, OUTPUT);

		isAttached = true;
	}

	return isAttached;
}

/*
 *  \brief effort of the motor, proportional to PWM
 *
 * @param effort a value from -1 to 1 representing effort
 *        0 is brake
 *        1 is full speed clockwise
 *        -1 is full speed counter clockwise
 */
void MotorBase::setEffort(float effort)
{
	if (effort > 1)
		effort = 1;
	if (effort < -1)
		effort = -1;

	currentEffort = effort;
}

/*
 * effort of the motor
 * @return a value from -1 to 1 representing effort
 *        0 is brake
 *        1 is full speed clockwise
 *        -1 is full speed counter clockwise
 */
float MotorBase::getEffort()
{
	return currentEffort;
}

/*
 * effort of the motor
 * @param effort a value from -1 to 1 representing effort
 *        0 is brake
 *        1 is full speed clockwise
 *        -1 is full speed counter clockwise
 */
void MotorBase::setEffortLocal(float effort)
{
	if(attach()) //returns true if already attached or if attached in the method
	{
		if(isReversed) effort *= -1;

		if (effort > 1)
			effort = 1;
		if (effort < -1)
			effort = -1;
		if (effort > 0)
			digitalWrite(directionPin, LOW);
		else
			digitalWrite(directionPin, HIGH);

		pwm.writeScaled(fabs(effort));
	}
}