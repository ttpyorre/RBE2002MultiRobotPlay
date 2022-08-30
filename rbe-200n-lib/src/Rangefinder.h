#pragma once

/*
 * Datasheet: https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 */

#include <Arduino.h>
#include <SPI.h>

#define ECHO_RECD   0x02
#define UART_RECD   0x04
#define ADC_READ    0x08

#define USE_CTRL_PIN    0x01
#define USE_ECHO        0x02
#define USE_UART        0x04
#define USE_ADC         0x08

class Rangefinder 
{
private:
    uint8_t state = 0;

    uint8_t echoPin = -1;
    uint8_t trigPin = -1;

    uint32_t lastPing = 0;          // for keeping track of intervals
    uint32_t pingInterval = 50;    // default to 200 ms

    uint32_t lastADCread = 0;       // can't read the ADC too fast

    uint32_t pulseStart = 0;
    uint32_t pulseEnd = 0;

    String serialString;
public:
    Rangefinder(uint8_t echo, uint8_t trig = -1);

    // void init(void);
    void init(uint8_t interfaces);

    //checks to see if it's time for a ping
    uint8_t checkPingTimer(void);

    //Checks to see if a pulse echo has been registered
    uint16_t checkEcho(void);

    //Reads the MCP3002 ADC; returns ADC result
    //Only works on the default SPI bus
    uint16_t readMCP3002(bool force = false);

    //Checks/reads on the RS-232 interface
    //Only works on Serial2
    uint16_t readASCII(void);

    //returns true and fills in distance ONLY when there is a new reading
    bool getDistance(float& distance);

    //ISR for the echo pins
    void ISR_echo(void);
};
