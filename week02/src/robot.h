#include <Chassis.h>
#include <Rangefinder.h>
#include <IRdecoder.h>
#include "hc-sr04.h"
#include "wallfollow.h"
#include "standoff.h"

//TODO: You'll want to make a wall_follower class to mimic the standoff
//#include "wall_follower.h" 

class Robot
{
protected:
    Chassis chassis;
    
    StandoffController standoffController;
    AveragingFilter filter;
    WallFollowing wallFollower;

    //Rangefinder hcsr04;
    Ultrasonic hc_sr04; //this is our ultrasonic, I think we can use it as we made it over using Lewin's

    IRDecoder irDecoder;
    
    enum ROBOT_STATE{   
                        ROBOT_IDLE, 
                        ROBOT_WALL_FOLLOWING, 
                        ROBOT_STANDOFF, 
                        ROBOT_DRIVE, 
                        ROBOT_DRIVE_TO_POINT,
                        DRIVE_STRAIGHT,
                        SPIN_CCW
                    };

    ROBOT_STATE robotState = ROBOT_IDLE;
    float rightSpeed;
    float leftSpeed;
    
public:
    Robot(void);
    void init(void);
    void loop(void);

protected:
    void handleIRPress(int16_t);
    void handleNewDistanceReading(float);
    void wallFollowing(float);
    void handleUpdateReady(void);
    void handleIMUtimer(void);
};