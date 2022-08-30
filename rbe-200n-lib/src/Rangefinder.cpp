#include "Rangefinder.h"

void IRAM_ATTR ISRforRangefinders(void * arg) 
{
	Rangefinder* obj = (Rangefinder*)arg;
	obj->ISR_echo();
}

// Constructor. Nothing to see here. Move along.
Rangefinder::Rangefinder(uint8_t echo, uint8_t trig) 
{
    echoPin = echo;
    trigPin = trig;
}

// Removed this so that you must call specific interfaces -- fewer troubles that way
// void Rangefinder::init(void)
// {
//     init(USE_ADC | USE_UART | USE_ECHO | USE_CTRL_PIN);
// }

// Allows the user to select the interface
void Rangefinder::init(uint8_t interfaces)
{
    if(interfaces & USE_ECHO)
    {
        // assert ECHO pin is an input
        pinMode(echoPin, INPUT);
        attachInterruptArg(digitalPinToInterrupt(echoPin), ISRforRangefinders, this, CHANGE);
    }

    if(interfaces & USE_UART)
    {
        //Serial2 is used for RS-232 format
        Serial2.begin(9600);
        Serial2.setRxInvert(true); // MaxBotix uses INVERTED logic levels
    }

    if(interfaces & USE_ADC) //uses the MCP3002, not an onboard ADC
    {
        //SPI to talk to the MCP3002
        SPI.begin(); //defaults to VPSI: SCK, MISO, MOSI, SS; see above
        pinMode(SS, OUTPUT); //need to set the CS to OUTPUT
    }

    if(interfaces & USE_CTRL_PIN)
    {
        //control pin for commanding pings
        pinMode(trigPin, OUTPUT);
    }
}

/**
 * checkPingTimer check to see if it's time to send a new ping.
 * You must select USE_CTRL_PIN in init() for this to work.
 */
uint8_t Rangefinder::checkPingTimer(void)
{
    //check if we're ready to ping
    if(millis() - lastPing >= pingInterval)
    {
        pulseEnd = pulseStart = 0;

        //clear out any leftover states
        state = 0;

        lastPing = millis();    //not perfectly on schedule, but safer and close enough
        lastADCread = lastPing; //this will make sure the proper interval is past before we read the ADC

        if(trigPin == -1) Serial.println("Invalid trigger pin.");

        digitalWrite(trigPin, HIGH); //commands a ping; leave high for the duration
        delayMicroseconds(30); //datasheet says hold HIGH for >20us; we'll use 30 to be 'safe'
        digitalWrite(trigPin, LOW); //unclear if pin has to stay HIGH
    }

    return state;
}

uint16_t Rangefinder::checkEcho(void)
{
    uint16_t echoLength = 0;
    if(state & ECHO_RECD)
    {
        echoLength = pulseEnd - pulseStart;
        state &= ~ECHO_RECD;
    }

    return echoLength;
}

uint16_t Rangefinder::readMCP3002(bool force)
{
    uint16_t retVal = 0;
    if((millis() - lastADCread >= 50) || force)
    {
        lastADCread = millis();

        // This will command the MCP to take a reading on CH0
        // Figure 6.1 of the datasheet shows the bit arrangement
        uint16_t cmdByte = 0x6800; 

        //start the SPI session
        SPISettings spiSettings; //defaults to (clk freq = 1000000, MSBFIRST, SPI_MODE0), which is what we want
        SPI.beginTransaction(spiSettings); 

        //open communication with the MCP3002
        digitalWrite(SS, LOW); 

        //this line both sends the command to read AND retrieves the result
        //the leading bits are indeterminate and need to be stripped off
        uint16_t ADCvalue = SPI.transfer16(cmdByte) & 0x03ff;

        //end communication
        digitalWrite(SS, HIGH); 

        //close the SPI session
        SPI.endTransaction(); 

        retVal = ADCvalue;
    }

    return retVal;
}

uint16_t Rangefinder::readASCII(void)
{
  while(Serial2.available())
  {
    char c = Serial2.read();

    if(c != 'R') serialString += c;

    if(c == 0xD) 
    {
      uint16_t result = serialString.toInt();
      serialString = "";
      return result;
    }
  }

  return 0;
}

//ISR for echo pin
void Rangefinder::ISR_echo(void)
{
    if(digitalRead(echoPin))  //transitioned to HIGH
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
bool Rangefinder::getDistance(float& distance)
{
    if(USE_CTRL_PIN) checkPingTimer();

    uint16_t pulseDur = checkEcho();
    if(pulseDur)
    {
        distance = pulseDur / 58.0;
        return true;
    }
    
    return false;
}