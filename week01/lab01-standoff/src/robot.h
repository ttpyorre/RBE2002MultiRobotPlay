#include <Chassis.h>
#include "wallfollow.h"
#include "standoff.h"
#include <IRdecoder.h>
#include "hc-sr04.h"

#define IR_PIN 15

class Robot
{
protected:
    StandoffController standoffController;
    WallFollowing wallFollower;

    Chassis chassis;
    IRDecoder irDecoder;
    Ultrasonic hc_sr04;

    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_WALL_FOLLOWING, ROBOT_STANDOFF};
    ROBOT_STATE robotState = ROBOT_IDLE;

public:
    Robot(void);
    void init(void);
    void loop(void);

protected:
    void handleIRPress(int16_t);
    void handleNewDistanceReading(float);
    void wallFollowing(float);
};