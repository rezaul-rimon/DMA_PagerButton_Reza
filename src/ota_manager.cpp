#include "ota_manager.h"
#include "main.h"
#include "config.h"
#include "network_manager.h"
#include "device_id.h"
#include <HTTPClient.h>
#include <Update.h>

void otaTask(void *param) {
    Serial.println("Starting OTA update...");

    HTTPClient http;
    http.begin(OTA_URL);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        int contentLength = http.getSize();
        Serial.printf("Content-Length: %d bytes\n", contentLength);

        if (Update.begin(contentLength)) {
            size_t written = Update.writeStream(http.getStream());
            if (written == contentLength) {
                Serial.println("OTA written successfully");
            }
            if (Update.end() && Update.isFinished()) {
                Serial.println("OTA update completed. Restarting...");
                NetworkManager::publish(MQTT_PUB_TOPIC, String(DeviceID::get()) + ",OTA update successful");
                vTaskDelay(pdMS_TO_TICKS(2000));
                ESP.restart();
            } else {
                Serial.println("OTA update failed!");
                NetworkManager::publish(MQTT_PUB_TOPIC, String(DeviceID::get()) + ",OTA Update Failed!");
            }
        } else {
            Serial.println("OTA begin failed!");
            NetworkManager::publish(MQTT_PUB_TOPIC, String(DeviceID::get()) + ",OTA Begin Failed!");
        }
    } else {
        Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
        NetworkManager::publish(MQTT_PUB_TOPIC, String(DeviceID::get()) + ",HTTP Request Failed");
    }

    http.end();
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();

    otaTaskHandle = NULL;
    vTaskDelete(NULL);
}