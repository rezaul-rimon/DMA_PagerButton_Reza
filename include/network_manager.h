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
    static void ensureWiFiConfigured();
    static void task(void *param);
    static void publish(const char* topic, const char* payload);
    static bool isWiFiConnected();
    static bool isMQTTConnected();
    static void markConfigured(bool value);   // NEW: used by reset button

private:
    static WiFiClient espClient;
    static PubSubClient mqttClient;

    static bool wifiCredentialsPresent();
    static bool flagSaysConfigured();
    static void setFlag(bool value);

    static void reconnectWiFi();
    static void reconnectMQTT();
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
    static void processMQTTQueue();
};

#endif