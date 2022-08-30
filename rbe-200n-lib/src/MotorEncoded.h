#pragma once

#include <Arduino.h>

/*
 * MotorEncoded.h
 *
 * Author: Greg Lewin, adapted from original work by hephaestus
 */

const uint32_t DRIVE_MOTOR_CTRL_INT_MS = 50;

#ifndef LIBRARIES_RBE200nLIB_SRC_MOTOR_ENCODED_H_
#define LIBRARIES_RBE200nLIB_SRC_MOTOR_ENCODED_H_

#include <MotorBase.h>
#include <PIDcontroller.h>

/** \brief A _derived_ motor class for motors with quadrature encoders. It is derived from
 * MotorBase and overrides the process() method, among others, to implement PID algorithms.
 * 
 * It was designed around the ESP32/Romi platform used in RBE 1001 and RBE 200n at WPI.
 * 
 * Currently only controls for speed (also allows for setting effort directly).
 * 
 * todo: add control for position and speed by position (red queen)
 * 
 */
class MotorEncoded : public MotorBase
{
protected:
	enum CTRL_MODE {CTRL_DIRECT, CTRL_SPEED, CTRL_POS, CTRL_SPD_WITH_POS};
	CTRL_MODE ctrlMode = CTRL_DIRECT;

	//for the Romi
	float encoderCountsPerMotorRev = 12.0;
	float gearSpeedRatio = 120.0;

	float degreesPerTick = 0; //calculated in the constructor

	uint32_t controlIntervalMS = DRIVE_MOTOR_CTRL_INT_MS;

private:
	/**
	 * GPIO pin number of the motor encoder A
	 */
	int MotorEncAPin = -1;

	/**
	 * GPIO pin number of the motor encoder B
	 */
	int MotorEncBPin = -1;

	// flag for whether or not the encoders are set up
	bool encodersEnabled = false;

	/**
	 * ESP32Encoder object to keep track of the motors position
	 */
	ESP32Encoder encoder;

	/**
	 * Variable to store the latest encoder read from the encoder hardware as read by the PID thread.
	 * This variable is set inside the PID thread, and read outside.
	 */
	int64_t currEncoder = 0;

	/*
	 * this stores the previous count of the encoder last time the velocity was calculated
	 */
	int64_t prevEncoder = 0;

	/**
	 * PID controller setpoint in encoder ticks per control interval
	 */
	float targetTicksPerInterval = 0; //really needs to be float?

	/**
	 * current speed, in ticks per interval, which is computationally easier than dps
	 */
	int64_t currTicksPerInterval = 0;

	/**
	 * PIDController object to perform PID control for speed.
	 * 
	 * todo: position control
	 */
	PIDController speedController;

public:
	MotorEncoded(int pwmPin, int dirPin, int encAPin, int encBPin);
	virtual ~MotorEncoded();

	/**
	 * Sets target speed.
	 */
	void setTargetDegreesPerSecond(float dps);

	float getDelta();
	float getDegreesPerSecond();
	float getCurrentDegrees();
	
	int64_t resetEncoder(void) {return prevEncoder = currEncoder;}

	/**
	 * Overloaded from MotorBase. Controls the effort to the motor directly.
	 * When called, it sets the control mode back to direct (none).
	 */
	void setEffort(float effort)
	{
		// when setEffort is called, we stop closed-loop control
		ctrlMode = CTRL_DIRECT;

		MotorBase::setEffort(effort);
	}
	
	/**
	 * Also overloaded from MotorBase. Controls the effort to the motor directly.
	 * When called, it sets the control mode back to direct (none).
	 */
	void setEffortPercent(float effort)
	{
		setEffort(effort / 100.0);
	}

    void setPIDgains(float kp, float ki, float kd, float cap = 0);

public:
	virtual bool attach(void);
	virtual void process();
};

#endif 
