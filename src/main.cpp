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
TaskHandle_t ledTaskHandle = NULL;   // <-- add this

QueueHandle_t rfQueue;
const char* DEVICE_ID = nullptr;

void setup() {
    Serial.begin(115200);
    DEBUG_PRINTLN("Booting...");

    DeviceID::init();
    DEVICE_ID = DeviceID::get();
    DEBUG_PRINT("Device ID: ");
    DEBUG_PRINTLN(DEVICE_ID);

    pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);

    LEDController::init();
    RFReceiver::init();

    rfQueue = xQueueCreate(20, sizeof(uint32_t));
    MQTTQueue::init();

    NetworkManager::init();
    LEDController::init();
    xTaskCreatePinnedToCore(LEDController::task, "LED", 2048, NULL, 2, &ledTaskHandle, 1);
    xTaskCreatePinnedToCore(networkTask, "Network", 8192, NULL, 1, &networkTaskHandle, 0);
    xTaskCreatePinnedToCore(mainTask, "Main", 8192, NULL, 1, &mainTaskHandle, 1);
    xTaskCreatePinnedToCore(wifiResetTask, "WiFiReset", 4096, NULL, 1, &wifiResetTaskHandle, 1);
    xTaskCreatePinnedToCore(RFReceiver::task, "RFReceiver", 4096, NULL, 1, &rfTaskHandle, 1);
}

void loop() {
}

void networkTask(void *param) {
    NetworkManager::task(param);
}

void mainTask(void *param) {
    for (;;) {
        uint32_t rfCode;
        if (xQueueReceive(rfQueue, &rfCode, 0) == pdTRUE) {
            String payload = String(DEVICE_ID) + "," + String(rfCode);
            NetworkManager::publish(MQTT_PUB_TOPIC, payload);
            DEBUG_PRINTLN("RF queued for MQTT: " + payload);
        }

        static unsigned long lastHeartbeat = 0;
        unsigned long now = millis();
        if (now - lastHeartbeat >= HB_INTERVAL_MS) {
            lastHeartbeat = now;
            if (NetworkManager::isMQTTConnected()) {
                String hb = String(DEVICE_ID) + ",wifi_connected";
                NetworkManager::publish(MQTT_HB_TOPIC, hb);
                DEBUG_PRINTLN("Heartbeat sent");
            }
        }
    }
}

void wifiResetTask(void *param) {
    for (;;) {
        if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
            unsigned long pressStart = millis();
            DEBUG_PRINTLN("Button pressed...");
            while (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
                if (millis() - pressStart >= 5000) {
                    DEBUG_PRINTLN("5s hold, starting WiFiManager...");
                    LEDController::setState(LedState::WIFI_AP_MODE);
                    vTaskSuspend(networkTaskHandle);
                    vTaskSuspend(mainTaskHandle);
                    vTaskSuspend(rfTaskHandle);
                    WiFiManager wm;
                    wm.resetSettings();
                    wm.autoConnect("DMA_Pager_Button");
                    ESP.restart();
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void rfTask(void *param) {
    vTaskDelete(NULL);
}