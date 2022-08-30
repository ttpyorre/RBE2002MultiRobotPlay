#include <Chassis.h>
#include <IRdecoder.h>
#include "hc-sr04.h"
#include "wallfollow.h"
#include "standoff.h"
#include "IRDirectionFinder.h"
#include <JulibotFollow.h>
#include <vector>

enum ROBOT_STATE
    {   
        //Fight Scene states
        ROMIO_INIT_FIGHT,
        ROMIO_FIGHT_STAGE, 
        ROMIO_RUN,

        TYBOT_INIT,
        TYBOT_CIRCLING,
        TYBOT_STAB,
        TYBOT_STAB_COMPLETE,
        TYBOT_DEAD,

        MERCUTIBOT_INIT,
        MERCUTIBOT_CIRCLING,
        MERCUTIBOT_WAITING_FOR_STAB,
        MERCUTIBOT_DEAD,

        //Balcony Scene states
        ROMIO_BALCONY_INIT,
        ROMIO_BALCONY_ENTER,
        ROMIO_BALCONY_FOLLOW,
        ROMIO_ENDING,
        
        JULIBOT_INIT,
        JULIBOT_CLIMB,
        JULIBOT_BROADCAST,
        JULIBOT_FOLLOW,
        JULIBOT_ENDING,
        
        FRIBOT_INIT,
        FRIBOT_ENTER,
        FRIBOT_ENDING,

        CURTAIN_CALL,
        CURTAIN_CALL_PERFORMANCE,
        CURTAIN_CALL_BACK,
        DOING_NOTHING,

        ROBOT_IDLE
    };

enum ROBOT_MQTT_STATE
    {
        MERCUTIBOT_AND_FRIBOT,
        TYBOT_AND_JULIBOT,
        ROMIO
    };

class Robot
{
public:

    //robotMQTTState determines which robot we are dealing with.
    ROBOT_MQTT_STATE robotMQTTState = MERCUTIBOT_AND_FRIBOT;
    //ROBOT_MQTT_STATE robotMQTTState = TYBOT_AND_JULIBOT;
    //ROBOT_MQTT_STATE robotMQTTState = ROMIO;

    //DEFAULT STATES !!
    ROBOT_STATE robotState = ROBOT_IDLE;
    //ROBOT_STATE robotState = CURTAIN_CALL;

protected:
    Chassis chassis;
    Pose currPose;
    AveragingFilter filter;
    Point point;

    IRDirectionFinder irFinder;
    IRDecoder irDecoder;
    Ultrasonic hc_sr04; //this is our Ultrasonic, code, we aren't using the rangefinder library code.

    JulibotFollow romio;
    WallFollowing wallFollower;
    StandoffController standoffController;

    double tybotError;
    double pitchAngle = 0;

    float rightSpeed, leftSpeed;
    float mercutibotDistance, julibotDistance;
    float distanceReading = 0;

    int circlingCount = 0;
    int startTime = 0, startBack = 0;
    int lastHandleUpdate = 0, lastHandleUpdateIMU = 0, lastHandleMQTT = 0, lastHandleLED = 0;

    bool initialize = false;
    bool startProgram = false;

    bool romioOnStage = false;
    bool haveWeSeenTybot = false;

    bool isClimbing = false;
    
    bool LEDOn = false;
    bool hasNewReading = false;

    vector<double> AccOld = {0, 0, 0};

public:
    Robot(void);
    void init(void);
    void loop(void);
    void handleNewDistanceReading(float);

    bool MercutibotIsReadyForStab = false;

    String castEnumToString();

protected:
    void handleIRPress(int16_t);

    void handleUpdateReady(void);
    void handleIMUtimer(void);
    void handleMQTTupdate(void);

    bool getInTheStage(int);
    bool ifIamStabbed(void);
    bool detectCollision(void);

    bool isOnStage = false;
    bool doneForward = false;
};