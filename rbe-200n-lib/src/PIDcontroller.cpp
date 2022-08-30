#include <PIDcontroller.h>
#include <math.h>

// float PIDController::CalcEffort(float current)
// {
//     float error = target - current;
//     return ComputeEffort(error);
// }

float PIDController::ComputeEffort(float error)
{
    currError = error; //store in case we want it later
    sumError += currError;

    if(errorBound > 0) //cap error; errorBound == 0 means don't cap
    {
        if(fabs(sumError) > errorBound) //you could multiply sumError by Ki to make it scale
        {
            sumError -= currError; //if we exceeded the limit, just subtract it off again
        }
    }

    float derivError = currError - prevError;
    prevError = currError;

    currEffort = Kp * currError + Ki * sumError + Kd * derivError;

    return currEffort;
}
