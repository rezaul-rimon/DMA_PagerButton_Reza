#include "ota_manager.h"
#include "main.h"
#include "config.h"
#include "network_manager.h"
#include "device_id.h"
#include <HTTPClient.h>
#include <Update.h>

void otaTask(void *param) {
    Serial.println("Starting OTA...");

    HTTPClient http;
    http.begin(OTA_URL);
    int httpCode = http.GET();

    char status[MQTT_PAYLOAD_MAX];

    if (httpCode == HTTP_CODE_OK) {
        int contentLength = http.getSize();
        Serial.printf("Content-Length: %d\n", contentLength);

        if (Update.begin(contentLength)) {
            size_t written = Update.writeStream(http.getStream());
            if (written == (size_t)contentLength) {
                Serial.println("OTA written");
            }
            if (Update.end() && Update.isFinished()) {
                Serial.println("OTA OK, restart");
                snprintf(status, sizeof(status), "%s,OTA update successful", getDeviceId());
                NetworkManager::publish(MQTT_PUB_TOPIC, status);
                vTaskDelay(pdMS_TO_TICKS(2000));
                ESP.restart();
            } else {
                snprintf(status, sizeof(status), "%s,OTA Update Failed!", getDeviceId());
                NetworkManager::publish(MQTT_PUB_TOPIC, status);
            }
        } else {
            snprintf(status, sizeof(status), "%s,OTA Begin Failed!", getDeviceId());
            NetworkManager::publish(MQTT_PUB_TOPIC, status);
        }
    } else {
        Serial.printf("HTTP failed: %s\n", http.errorToString(httpCode).c_str());
        snprintf(status, sizeof(status), "%s,HTTP Request Failed", getDeviceId());
        NetworkManager::publish(MQTT_PUB_TOPIC, status);
    }

    http.end();
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();

    otaTaskHandle = NULL;
    vTaskDelete(NULL);
}