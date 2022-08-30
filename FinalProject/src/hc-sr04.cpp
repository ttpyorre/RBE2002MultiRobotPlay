#include "hc-sr04.h"

#define US_CTRL 23     //for pinging -- not a great choice since this can hamper uploading || 16 normally, ours broke | 23 Green
#define PULSE_PIN 19    //for reading pulse                                                  || 17 normally             | 19 Yellow

#define US_WINDOW_DUR 100    //ms

Ultrasonic hc_sr04;


//ISR for reading the echoes. Or is it echos?
void ISR_Ultrasonic(void)
{
    hc_sr04.US_ISR();
}

// Constructor. Nothing to see here. Move along.
Ultrasonic::Ultrasonic(void) {}


// Default init engages all interfaces
void Ultrasonic::init(void)
{
    init(USE_ECHO | USE_CTRL_PIN);
}

// Allows the user to select the interface
void Ultrasonic::init(uint8_t interfaces)
{
    if(interfaces & USE_ECHO)
    {
        // assert ECHO pin is an input
        pinMode(PULSE_PIN, INPUT);
        attachInterrupt(PULSE_PIN, ISR_Ultrasonic, CHANGE);
    }

    if(interfaces & USE_CTRL_PIN)
    {
        //control pin for commanding pings
        pinMode(US_CTRL, OUTPUT);
    }
}

/**
 * checkPingTimer check to see if it's time to send a new ping.
 * You must select USE_CTRL_PIN in init() for this to work.
 */
uint8_t Ultrasonic::checkPingTimer(void)
{
    //check if we're ready to ping
    if(millis() - lastPing >= pingInterval)
    {
        pulseEnd = pulseStart = 0;

        //clear out any leftover states
        state = 0;

        lastPing = millis();    //not perfectly on schedule, but safer and close enough
        //lastADCread = lastPing; //this will make sure the proper interval is past before we read the ADC

        digitalWrite(US_CTRL, HIGH); //commands a ping; leave high for the duration
        delayMicroseconds(30); //datasheet says hold HIGH for >20us; we'll use 30 to be 'safe'
        digitalWrite(US_CTRL, LOW); //unclear if pin has to stay HIGH
    }

    return state;
}

uint16_t Ultrasonic::checkEcho(void)
{
    uint16_t echoLength = 0;
    if(state & ECHO_RECD)
    {
        echoLength = pulseEnd - pulseStart;
        state &= ~ECHO_RECD;
    }

    return echoLength;
}

//ISR for echo pin
void Ultrasonic::US_ISR(void)
{
    if(digitalRead(PULSE_PIN))  //transitioned to HIGH
    {
        pulseStart = micros();
    }

    else                        //transitioned to LOW
    {
        pulseEnd = micros();
        state |= ECHO_RECD;
    } 
}

/**
 * TODO: Write a getDistance() function for the distance method of your choice.
 * 
 * getDistance should return true whenever there is a new reading, and put the result
 * in distance, which is _passed by reference_ so that you can "return" a value
 */
bool Ultrasonic::getDistance(float& distance)
{      
    uint32_t pulseLen = hc_sr04.checkEcho();
    if(pulseLen)
    {
        distance = pulseLen;
      
        // y = mx + b, these are the values we got and that we have to calculate in code to get x, that is the distance.
        // m = 55, b = 43, y = pulseWidth
        distance -= 43;
        distance /= 55;

        return true;
    }
    else
    {
        return false;
    }
}
