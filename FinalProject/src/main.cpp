#include <Arduino.h>
#include <RBE-200n-Lib.h>

#include "robot.h"
#include "mqtt.h"

Robot robot;

void mqttCallback(char* topic, byte *payload, unsigned int length);

void setup() 
{
  Serial.begin(115200);
  
  delay(500);
  
  Wire.begin();
  Wire.setClock(100000ul);

  Serial.println(F("Welcome."));

  robot.init();

  setup_mqtt(); // also calls setup_wifi() 
  reconnect();
  
  client.setCallback(mqttCallback); //mqttCallback

  //We will subscribe to all states, then depending what our current robot is, we will use only those subscribtions
  client.subscribe("team11/Romio/State"); 
  client.subscribe("team11/Tybot&Julibot/State"); 
  client.subscribe("team11/Mercutibot&Fribot/State");
}

void loop()
{
  robot.loop();
}

void mqttCallback(char* topic, byte *payload, unsigned int length)
{
  //Subscribe to team11/Romio/Signal
  //This location sends signals qwerty(Romio Fight) | QWERTY(Romio Balcony)
  if(String(topic) == "team11/Romio/State")
  {
    //So if the Romi thinks its Tybot or Julibot
    if(length && robot.robotMQTTState == TYBOT_AND_JULIBOT)
    {
      //Julibot receiving from Romio Balcony
      //Julibot sees that Romio is at state ROMIO_BALCONY_FOLLOW, Julibot is broadcasting, so she should change to follow
      if(payload[0] == 'E' && robot.robotState == JULIBOT_BROADCAST) robot.robotState = JULIBOT_FOLLOW;
    }
  }

  //Subscribe to team11/Tybot&Julibot/Signal
  //This location sends signals asdfg(Tybot) | ASDFG(Julibot)
  if(String(topic) == "team11/Tybot&Julibot/State")
  {
    if(length && robot.robotMQTTState == MERCUTIBOT_AND_FRIBOT)
    {
      //Mercutibot receiving from Tybot
      if(payload[0] == 'a') robot.robotState = MERCUTIBOT_INIT;
      if(payload[0] == 'd') robot.robotState = MERCUTIBOT_DEAD;

      //Fribot receiving from Julibot
      if(payload[0] == 'A') robot.robotState = FRIBOT_INIT;
      if(payload[0] == 'F') robot.robotState = FRIBOT_ENTER;

      //Curtain Call
      if(payload[0] == 'J') robot.robotState = CURTAIN_CALL;

    }
    if(length && robot.robotMQTTState == ROMIO)
    {
      //Romio Fight receiving from Tybot
      if(payload[0] == 'a') robot.robotState = ROMIO_INIT_FIGHT;
      if(payload[0] == 'd' && robot.robotState == ROMIO_INIT_FIGHT) robot.robotState = ROMIO_FIGHT_STAGE;
      if(payload[0] == 'g' && robot.robotState == ROMIO_FIGHT_STAGE) robot.robotState = ROMIO_RUN;


      //Romio Balcony receiving from Julibot
      if(payload[0] == 'A') robot.robotState = ROMIO_BALCONY_INIT;
      if(payload[0] == 'D' && robot.robotState == ROMIO_BALCONY_INIT) robot.robotState = ROMIO_BALCONY_ENTER;
      if(payload[0] == 'F' && robot.robotState == ROMIO_BALCONY_ENTER) robot.robotState = ROMIO_BALCONY_FOLLOW;
      if(payload[0] == 'G' && robot.robotState == ROMIO_BALCONY_FOLLOW) robot.robotState = ROMIO_ENDING;

      //Curtain Call
      if(payload[0] == 'J') robot.robotState = CURTAIN_CALL;
    }
  }
  
  //Subscribe to team11/Mercutibot&Fribot/Signal
  //This location sends signals zxcvb(Mercutibot) | ZXCVB(Fribot)
  if(String(topic) == "team11/Mercutibot&Fribot/State")
  {
    if(length && robot.robotMQTTState == TYBOT_AND_JULIBOT)
    {
      //Tybot receiving from Mercutibot
      //'c' translates into received other robot state being MERCUTIBOT_WAITING_FOR_STAB
      if(payload[0] == 'c' && robot.robotState == TYBOT_CIRCLING)
      {
        robot.robotState = TYBOT_STAB;
        robot.MercutibotIsReadyForStab = false;
      }
    }
    if(length && robot.robotMQTTState == ROMIO)
    {
      if(payload[0] == 'E') robot.robotState = JULIBOT_FOLLOW;
    }
  }
  
}