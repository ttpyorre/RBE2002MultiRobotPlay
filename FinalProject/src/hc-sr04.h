#ifndef __HC_SR04_H
#define __HC_SR04_H

/*
 * Datasheet: https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 */

#include <Arduino.h>
#include <SPI.h>

#define ECHO_RECD   0x02

#define USE_CTRL_PIN    0x01
#define USE_ECHO        0x02

class Ultrasonic
{
private:
    uint8_t state = 0;

    uint32_t lastPing = 0;          // for keeping track of intervals
    uint32_t pingInterval = 50;    // default to 200 ms

    uint32_t pulseStart = 0;
    uint32_t pulseEnd = 0;

    String serialString;
    
public:
    Ultrasonic(void);  //ideally, this would take pins as parameters, but just hard-coded for now since we only have one
    void init(void);
    void init(uint8_t interfaces);

    //checks to see if it's time for a ping
    uint8_t checkPingTimer(void);

    //Checks to see if a pulse echo has been registered
    uint16_t checkEcho(void);

    /**
     * TODO: Write a getDistance() function for the distance method of your choice.
     * 
     * See the .cpp file.
     */
    bool getDistance(float& distance);

    //ISR for the MaxBotix sensor
    void US_ISR(void);
};


#endif