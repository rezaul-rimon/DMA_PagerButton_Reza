#include "mqtt_queue.h"

QueueHandle_t MQTTQueue::queue = nullptr;

void MQTTQueue::init() {
    queue = xQueueCreate(20, sizeof(String*));
}

bool MQTTQueue::enqueue(const String& message) {
    if (!queue) return false;
    String* msg = new String(message);
    if (xQueueSend(queue, &msg, 0) != pdTRUE) {
        delete msg;
        return false;
    }
    return true;
}

bool MQTTQueue::dequeue(String& message, TickType_t timeout) {
    if (!queue) return false;
    String* msg = nullptr;
    if (xQueueReceive(queue, &msg, timeout) == pdTRUE) {
        message = *msg;
        delete msg;
        return true;
    }
    return false;
}