#ifndef MQTT_QUEUE_H
#define MQTT_QUEUE_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <Arduino.h>
#include "config.h"

struct MqttMessage {
    char topic[MQTT_TOPIC_MAX];
    char payload[MQTT_PAYLOAD_MAX];
};

namespace MQTTQueue {
    void init();
    bool enqueue(const char* topic, const char* payload);
    bool dequeue(MqttMessage* out, TickType_t timeout = 0);
}

#endif