/*
 * MotorBase.h
 *
 * Author: Greg Lewin, adapted from original work by hephaestus
 */

#ifndef LIBRARIES_RBE200nLIB_SRC_MOTOR_BASE_H_
#define LIBRARIES_RBE200nLIB_SRC_MOTOR_BASE_H_

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

/** \brief A base class for motors, with pwm controlled by an ESP32PWM object.
 *
 * The MotorBase class implements pwm for effort (including direction). Several methods are declared virtual
 * and overridden by derived classes (currently just MotorEncoded).
 *
 * The class uses one timer for all of the ESP32PWM objects, which is set up in Chassis.
 * 
 * process() is called from the interrupt routine for each motor. 
 * process() is virtual; derived classes can implement their specific functionality.
 */
class MotorBase
{
private:
	/**
	 * GPIO pin number of the motor PWM pin
	 */
	int MotorPWMPin = -1;

	/**
	 * GPIO pin number of the motor direction output flag
	 */
	int directionPin = -1;

	/**
	 * True if the motor has been attached
	 */
	bool isAttached = false;

	/**
	 * variable for caching the current effort being sent to the PWM/direction pins
	 */
	float currentEffort = 0;

	/**
	 * the object that produces PWM for motor speed
	 */
	ESP32PWM pwm;

public:
	MotorBase(int pwmPin, int dirPin);
	virtual ~MotorBase();

protected:
	/**
	 * \brief Setup of hardware and register the motor.
	 *
	 * This attaches the motors to the hardware ports that were saved in the constructor.
	 * @note this must only be called after timers are allocated via Motor::allocateTimers(int PWMgenerationTimer)
	 *
	 * todo: verify/eliminate the note above
	 */
	virtual bool attach(void);

public:
	/*
	 *  \brief effort of the motor, proportional to PWM
	 *
	 * @param effort a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 */
	virtual void setEffort(float effort);
	
	/*
	 * set the effort of the motor in percent
	 * @param percent a value from -100 to 100 representing effort
	 *        0 is brake
	 *        100 is full speed clockwise
	 *        -100 is full speed counter clockwise
	 */
	virtual void setEffortPercent(float percent)
	{
		setEffort(percent * 0.01);
	}

	/*
	 * effort of the motor
	 * @return a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 */
	float getEffort();

	/*
	 * effort of the motor
	 * @return a value from -100 to 100 representing effort
	 *        0 is brake
	 *        100 is full speed clockwise
	 *        -100 is full speed counter clockwise
	 */
	float getEffortPercent()
	{
		return getEffort() * 100;
	}

	// sets the motor so that positive is anti-clockwise
	bool setReverse(bool rev = true) {return isReversed = rev;}

protected:

	bool isReversed = false;

	/**
	 * Called each time through the control loop (from Chassis)
	 * Overridden by derived classes to implement control methods.
	 */
	virtual void process(void);

private:
	/* Sets the nitty-gritty of the motor. Only called from process().

	 * effort of the motor
	 * @param a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 * @note this should only be called from the PID thread
	 */
	void setEffortLocal(float effort);
};

#endif 
