#pragma once

#include <Arduino.h>

class PIDController
{
private:
    //float target = 0;

    float Kp, Ki, Kd;
    float currError = 0;
    float prevError = 0;
    
    float sumError = 0;
    float errorBound = 0;

    float deltaT = 0; //not used for now; could be useful

    float currEffort = 0;
public:
    static volatile uint8_t readyToPID;   //a flag that is set when the PID timer overflows

public:
    PIDController(float p, float i = 0, float d = 0, float bound = 0) : Kp(p), Ki(i), Kd(d), errorBound(bound) {}
    float ComputeEffort(float error);
    float computeEffort(float error) {return ComputeEffort(error);}
    //float CalcEffort(float current);
    float setKp(float k) {return Kp = k;}
    float setKi(float k) {sumError = 0; return Ki = k;}
    float setKd(float k) {return Kd = k;}
    float setCap(float cap) {return errorBound = cap;}
};
