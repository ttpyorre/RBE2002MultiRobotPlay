#include <PIDcontroller.h>
#include <openmv.h>


class JulibotFollow
{
public:
    float leftEffort = 0;
    float rightEffort = 0;
    int lastHandleUpdateJulibot = 0;
    int robotEffort = 10;

protected:
    float targetDistance = 50; //in cm
    float targetX = 80;
    float robotSpeed = 10;

    int cx, cy, width, height, rot;

    PIDController piStandoff;
    PIDController piStandoffTurn;

public:
    JulibotFollow(void) : piStandoff(0.6, 0, 0.05), piStandoffTurn(0.25, 0, 1) {}
    double getJulibotDistance();
    void followingJulibot(float distance);
    void tagStandoff();
    uint8_t FindAprilTags();
};