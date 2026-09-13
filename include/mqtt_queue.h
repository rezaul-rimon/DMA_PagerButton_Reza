#ifndef MQTT_QUEUE_H
#define MQTT_QUEUE_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <Arduino.h>

class MQTTQueue {
public:
    static void init();
    static bool enqueue(const String& message);
    static bool dequeue(String& message, TickType_t timeout = 0);
private:
    static QueueHandle_t queue;
};

#endif