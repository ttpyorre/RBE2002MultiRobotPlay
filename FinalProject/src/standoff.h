#include <PIDcontroller.h>
#include <filter.h>

class StandoffController
{
public:
    float leftEffort = 0;
    float rightEffort = 0;

protected:
    float targetDistance = 30;

    PIDController piStandoffer;

public:
    StandoffController(void) : piStandoffer(2, 0, 0) {}

    void processDistanceReading(float distance);
    void handleKeyPress(int16_t key);
};