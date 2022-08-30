#include <PIDcontroller.h>

class WallFollowing
{
public:
    float leftEffort = 0;
    float rightEffort = 0;
    

protected:
    float targetDistance = 25; //in cm
    float robotSpeed = 15;     //in cm, TODO: Modify robot speed with use of effor and this value to make the robot go 25cm/s
    float robotSpeedBackwards = -15;

    PIDController piWallFollowing;

public:
    WallFollowing(void) : piWallFollowing(0.9, 0, 2) {} //TODO: edit gains: these values work pretty smooth tho 0.9, 0, 2

    void processDistanceReading(float distance);
    void handleKeyPress(int16_t key);
};