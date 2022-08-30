#include "standoff.h"
#include "ir_codes.h"

//Bases the effort for wheels based on the distance away from target.
void StandoffController::processDistanceReading(float distance)
{
    float error = distance - targetDistance;
    float effort = piStandoffer.computeEffort(error);

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
            if(key >= NUM_0 && key <= NUM_9)
            {
                //The keys are conveniently mapped so we can get the value we want with this math, by pressing a specified key.
                targetDistance = key % 16;
                targetDistance *= 10;
            }
            break;
    }
}