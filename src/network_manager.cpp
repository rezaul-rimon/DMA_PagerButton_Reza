#include "network_manager.h"
#include "main.h"
#include "led_controller.h"
#include "mqtt_queue.h"
#include "device_id.h"

WiFiClient NetworkManager::espClient;
PubSubClient NetworkManager::mqttClient(espClient);

bool NetworkManager::isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool NetworkManager::isMQTTConnected() {
    return mqttClient.connected();
}

void NetworkManager::init() {
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
}

void NetworkManager::reconnectWiFi() {
    static int attempt = 0;
    static int wait = 0;
    static int cycle = MAX_WIFI_ATTEMPTS;

    if (WiFi.status() != WL_CONNECTED) {
        if (attempt < WIFI_ATTEMPT_COUNT) {
            DEBUG_PRINTLN("Attempting WiFi connection...");
            WiFi.begin();
            attempt++;
            vTaskDelay(pdMS_TO_TICKS(WIFI_ATTEMPT_DELAY_MS));
        } else if (wait < WIFI_WAIT_COUNT) {
            DEBUG_PRINTLN("Waiting for WiFi...");
            wait++;
            vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_DELAY_MS));
        } else {
            attempt = 0;
            wait = 0;
            cycle--;
            if (cycle <= 0) {
                DEBUG_PRINTLN("Max WiFi cycles reached, restarting...");
                ESP.restart();
            }
        }
    } else {
        attempt = 0;
        wait = 0;
        cycle = MAX_WIFI_ATTEMPTS;
    }
}

void NetworkManager::reconnectMQTT() {
    static int attempt = 0;
    if (!mqttClient.connected()) {
        if (attempt < MQTT_ATTEMPT_COUNT) {
            char clientId[24];
            snprintf(clientId, sizeof(clientId), "dma_pgb_%04X%04X%04X", random(0xffff), random(0xffff), random(0xffff));

            DEBUG_PRINTLN("Attempting MQTT connection...");
            if (mqttClient.connect(clientId, MQTT_USER, MQTT_PASSWORD)) {
                LEDController::blinkColor(CRGB::Green, 2, 300);
                DEBUG_PRINTLN("MQTT connected");
                DEBUG_PRINT("Client_ID: ");
                DEBUG_PRINTLN(clientId);
                attempt = 0;

                String subTopic = String(MQTT_SUB_TOPIC) + "/" + DeviceID::get();
                mqttClient.subscribe(subTopic.c_str());
            } else {
                DEBUG_PRINTLN("MQTT connection failed");
                attempt++;
                vTaskDelay(pdMS_TO_TICKS(MQTT_ATTEMPT_DELAY_MS));
            }
        } else {
            DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting...");
            ESP.restart();
        }
    }
}

void NetworkManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    DEBUG_PRINTLN("Message arrived on topic: " + String(topic));
    DEBUG_PRINTLN("Message content: " + message);

    if (message == "ping") {
        String pingData = String(DeviceID::get()) + "," + WiFi.SSID() + "," + WiFi.localIP().toString() + "," + String(WiFi.RSSI()) + "," + String(HB_INTERVAL_MS);
        MQTTQueue::enqueue(String(MQTT_PUB_TOPIC) + ":" + pingData);
    } else if (message == "update_firmware") {
        if (otaTaskHandle == NULL) {
            xTaskCreatePinnedToCore(otaTask, "OTA Task", 8192, NULL, 1, &otaTaskHandle, 1);
        } else {
            Serial.println("OTA Task already running.");
        }
    }
}

void NetworkManager::processMQTTQueue() {
    String queuedMessage;
    if (MQTTQueue::dequeue(queuedMessage, 0)) {
        int sep = queuedMessage.indexOf(':');
        if (sep != -1) {
            String topic = queuedMessage.substring(0, sep);
            String payload = queuedMessage.substring(sep + 1);
            if (mqttClient.publish(topic.c_str(), payload.c_str())) {
                DEBUG_PRINTLN("Published: " + payload + " to " + topic);
                LEDController::blinkColor(CRGB::Green, 1, 250);
            } else {
                DEBUG_PRINTLN("MQTT publish failed");
            }
        }
    }
}

void NetworkManager::publish(const String& topic, const String& payload) {
    MQTTQueue::enqueue(topic + ":" + payload);
}

void NetworkManager::task(void *param) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mqttClient.connected()) {
                reconnectMQTT();
                LEDController::setState(LedState::MQTT_OFFLINE);
            } else {
                // Both WiFi and MQTT connected → base off
                LEDController::setState(LedState::OFF);
                mqttClient.loop();
                processMQTTQueue();
            }
        } else {
            reconnectWiFi();
            LEDController::setState(LedState::WIFI_DISCONNECTED);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}