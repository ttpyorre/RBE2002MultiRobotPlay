#include "standoff.h"
#include "ir_codes.h"

void StandoffController::processDistanceReading(float distance)
{
    float error = targetDistance - distance;
    float effort = piStandoffer.ComputeEffort(-error);
    
    leftEffort = effort;
    rightEffort = effort;
}


void StandoffController::handleKeyPress(int16_t key)
{
    switch(key)
    {
        case CHplus:
            targetDistance += 10;
            break;

        case CHminus:
            targetDistance -= 10;
            break;
        

        default:
            if(key >= NUM_1 && key <= NUM_9)
            {
                //Distance will be (key modulo 16 * 10cm).
                targetDistance = key % 16;
                targetDistance *= 10;
            }
            break;
    }
}