#pragma once

#include "MotorBase.h"

// Pins used by a perpheral, may be re-used
#define BOOT_FLAG_PIN 0

// serial busses
#define I2C_SDA 21
#define I2C_SCL 22

#define SERIAL_PROGRAMMING_TX 1
#define SERIAL_PROGRAMMING_RX 3

#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18
#define SPI_SS 5

// Ultrasonics
#define SIDE_ULTRASONIC_TRIG 16
#define SIDE_ULTRASONIC_ECHO 17

#define LEFT_LINE_SENSE 36
#define RIGHT_LINE_SENSE 39

#define SERVO_PIN 33
#define SERVO_FEEDBACK_SENSOR 34

#define MOTOR_DISABLE 15 //does this actually do anything?

#define MOTOR_LEFT_PWM 13
#define MOTOR_LEFT_DIR 4
#define MOTOR_LEFT_ENCA 26
#define MOTOR_LEFT_ENCB 27

#define MOTOR_RIGHT_PWM 12
#define MOTOR_RIGHT_DIR 25
#define MOTOR_RIGHT_ENCA 14
#define MOTOR_RIGHT_ENCB 32

