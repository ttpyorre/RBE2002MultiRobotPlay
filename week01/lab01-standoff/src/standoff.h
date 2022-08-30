#include <PIDcontroller.h>

class StandoffController
{
public:
    float leftEffort = 0;
    float rightEffort = 0;

protected:
    float targetDistance = 50;

    PIDController piStandoffer;

public:
    StandoffController(void) : piStandoffer(1, 0, 0) {} //TODO: edit gains to smth better

    void processDistanceReading(float distance);
    void handleKeyPress(int16_t key);
};