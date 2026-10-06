#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "config.h"
#include "main.h"
#include "device_id.h"
#include "led_controller.h"
#include "network_manager.h"
#include "rf_receiver.h"
#include "mqtt_queue.h"
#include "ota_manager.h"
#include <WiFiManager.h>

TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t wifiResetTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;
TaskHandle_t rfTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

QueueHandle_t rfQueue;

void setup() {
    Serial.begin(115200);
    DEBUG_PRINTLN("Booting...");

    randomSeed(esp_random());

    initDeviceId();
    DEBUG_PRINT("Device ID: ");
    DEBUG_PRINTLN(getDeviceId());

    pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);

    LEDController::init();
    RFReceiver::init();

    rfQueue = xQueueCreate(20, sizeof(uint32_t));
    MQTTQueue::init();

    NetworkManager::init();

    // ---- Start LED task FIRST so it can render AP mode ----
    xTaskCreatePinnedToCore(LEDController::task, "LED", 2048, NULL, 2, &ledTaskHandle, 1);
    vTaskDelay(pdMS_TO_TICKS(50));   // let the LED task begin running

    // ---- Now check WiFi; may block in AP portal with LED already active ----
    NetworkManager::ensureWiFiConfigured();

    // ---- Start remaining tasks ----
    xTaskCreatePinnedToCore(networkTask,        "Network", 8192, NULL, 1, &networkTaskHandle,   0);
    xTaskCreatePinnedToCore(mainTask,           "Main",    8192, NULL, 1, &mainTaskHandle,      1);
    xTaskCreatePinnedToCore(wifiResetTask,      "WiFiRst", 4096, NULL, 1, &wifiResetTaskHandle, 1);
    xTaskCreatePinnedToCore(RFReceiver::task,   "RF",      4096, NULL, 1, &rfTaskHandle,        1);

    DEBUG_PRINT("Free heap at boot: ");
    DEBUG_PRINTLN(ESP.getFreeHeap());
}

void loop() {
    // Empty, all in tasks
}

void networkTask(void *param) {
    NetworkManager::task(param);
}

void mainTask(void *param) {
    for (;;) {
        // 1) Drain RF queue -> MQTT queue
        uint32_t rfCode;
        while (xQueueReceive(rfQueue, &rfCode, 0) == pdTRUE) {
            char payload[MQTT_PAYLOAD_MAX];
            snprintf(payload, sizeof(payload), "%s,%lu", getDeviceId(), (unsigned long)rfCode);
            NetworkManager::publish(MQTT_PUB_TOPIC, payload);
            DEBUG_PRINT("RF->MQTT: ");
            DEBUG_PRINTLN(payload);
        }

        // 2) Heartbeat
        static unsigned long lastHeartbeat = 0;
        unsigned long now = millis();
        if (now - lastHeartbeat >= HB_INTERVAL_MS) {
            lastHeartbeat = now;
            if (NetworkManager::isMQTTConnected()) {
                char hb[MQTT_PAYLOAD_MAX];
                snprintf(hb, sizeof(hb), "%s,wifi_connected", getDeviceId());
                NetworkManager::publish(MQTT_HB_TOPIC, hb);
                DEBUG_PRINTLN("Heartbeat sent");

                DEBUG_PRINT("Free heap: ");
                DEBUG_PRINT(ESP.getFreeHeap());
                DEBUG_PRINT(" | Min free: ");
                DEBUG_PRINTLN(ESP.getMinFreeHeap());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void wifiResetTask(void *param) {
    for (;;) {
        if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
            unsigned long pressStart = millis();
            DEBUG_PRINTLN("Button pressed...");
            while (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
                if (millis() - pressStart >= 5000) {
                    DEBUG_PRINTLN("5s hold -> clearing WiFi config + AP portal");

                    // 1. Clear our "configured" flag FIRST so next boot
                    //    definitely enters AP mode even if portal crashes
                    NetworkManager::markConfigured(false);

                    // 2. LED feedback
                    LEDController::setState(LedState::WIFI_AP_MODE);
                    vTaskDelay(pdMS_TO_TICKS(200));

                    // 3. Suspend other tasks
                    vTaskSuspend(networkTaskHandle);
                    vTaskSuspend(mainTaskHandle);
                    vTaskSuspend(rfTaskHandle);

                    // 4. Wipe WiFi credentials & run portal
                    WiFiManager wm;
                    wm.setConfigPortalTimeout(AP_PORTAL_TIMEOUT_SEC);
                    wm.setBreakAfterConfig(true);
                    wm.resetSettings();

                    bool ok = wm.startConfigPortal(AP_PORTAL_SSID);

                    // 5. If user configured, mark it
                    if (ok && WiFi.status() == WL_CONNECTED) {
                        NetworkManager::markConfigured(true);
                        WiFi.persistent(true);
                        WiFi.begin();
                        vTaskDelay(pdMS_TO_TICKS(1500));
                    }

                    ESP.restart();
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}