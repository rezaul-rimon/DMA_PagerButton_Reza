#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#undef min
#undef max
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

class NetworkManager {
public:
    static void init();
    static void task(void *param);
    static void publish(const String& topic, const String& payload);
    static bool isWiFiConnected();
    static bool isMQTTConnected();
private:
    static WiFiClient espClient;
    static PubSubClient mqttClient;
    static void reconnectWiFi();
    static void reconnectMQTT();
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
    static void processMQTTQueue();
};

#endif