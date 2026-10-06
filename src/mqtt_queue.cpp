#include "mqtt_queue.h"

static QueueHandle_t s_queue = nullptr;

void MQTTQueue::init() {
    s_queue = xQueueCreate(MQTT_QUEUE_DEPTH, sizeof(MqttMessage));
}

bool MQTTQueue::enqueue(const char* topic, const char* payload) {
    if (!s_queue) return false;

    MqttMessage msg;
    strncpy(msg.topic, topic, sizeof(msg.topic) - 1);
    msg.topic[sizeof(msg.topic) - 1] = '\0';
    strncpy(msg.payload, payload, sizeof(msg.payload) - 1);
    msg.payload[sizeof(msg.payload) - 1] = '\0';

    return xQueueSend(s_queue, &msg, 0) == pdTRUE;
}

bool MQTTQueue::dequeue(MqttMessage* out, TickType_t timeout) {
    if (!s_queue || !out) return false;
    return xQueueReceive(s_queue, out, timeout) == pdTRUE;
}