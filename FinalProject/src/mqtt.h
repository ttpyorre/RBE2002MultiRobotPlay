#pragma once

#if defined(ESP8266)
#include <ESP8266WiFi.h>  //ESP8266 Core WiFi Library
#elif defined(ESP32)
#include <WiFi.h>         //ESP32 Core WiFi Library
#else
#include <WiFi.h>
#endif

#include <PubSubClient.h>

void setup_wifi();
void wifi_reconnect(void);
void reconnect();
void callback(char* topic, byte *payload, unsigned int length);
void setup_mqtt();
//void mqttCallback(char* topic, byte *payload, unsigned int length);

extern WiFiClient wifiClient;
extern PubSubClient client;
