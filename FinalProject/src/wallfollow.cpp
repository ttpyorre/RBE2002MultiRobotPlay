#include "wallfollow.h"
#include "ir_codes.h"

void WallFollowing::processDistanceReading(float distance)
{
    float error = targetDistance - distance;
    float effort = piWallFollowing.ComputeEffort(-error);
    
    rightEffort = -effort + robotSpeed;
    leftEffort  =  effort + robotSpeed;
}

void WallFollowing::handleKeyPress(int16_t key)
{
    switch(key)
    {
        case CHplus:
            targetDistance += 10;
            break;

        case CHminus:
            targetDistance -= 10;
            break;

        case PREV:
            robotSpeed += 5;
            break;

        case NEXT:
            robotSpeed -= 5;
            break;

        default:
            if(key >= NUM_1 && key <= NUM_9)
            {
                // The keys are conveniently mapped so we can get the value we want with this math.
                targetDistance = key % 16;
                targetDistance *= 10;
            }
            break;
    }
}