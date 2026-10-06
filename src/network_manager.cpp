#include "network_manager.h"
#include "main.h"
#include "led_controller.h"
#include "mqtt_queue.h"
#include "device_id.h"
#include "ota_manager.h"
#include <WiFiManager.h>
#include <Preferences.h>
#include <nvs.h>

WiFiClient NetworkManager::espClient;
PubSubClient NetworkManager::mqttClient(espClient);

// Our own namespace & key
static const char* NET_CFG_NS  = "net_cfg";
static const char* NET_CFG_KEY = "configured";

// -----------------------------------------------------------------
// Our own flag helpers
// -----------------------------------------------------------------

bool NetworkManager::flagSaysConfigured() {
    Preferences p;
    if (!p.begin(NET_CFG_NS, true)) return false;
    bool v = p.getBool(NET_CFG_KEY, false);
    p.end();
    return v;
}

void NetworkManager::setFlag(bool value) {
    Preferences p;
    if (!p.begin(NET_CFG_NS, false)) return;
    p.putBool(NET_CFG_KEY, value);
    p.end();
    DEBUG_PRINT("net_cfg.configured = ");
    DEBUG_PRINTLN(value ? "true" : "false");
}

// Public — used by wifiResetTask
void NetworkManager::markConfigured(bool value) {
    setFlag(value);
}

// -----------------------------------------------------------------
// Legacy migration: try to read ESP-IDF NVS once
// -----------------------------------------------------------------

bool NetworkManager::wifiCredentialsPresent() {
    // 1) Our own flag is authoritative
    if (flagSaysConfigured()) {
        DEBUG_PRINTLN("Creds flag: TRUE");
        return true;
    }

    // 2) Fallback: try ESP-IDF NVS once (handles upgrade from old firmware)
    nvs_handle_t h;
    if (nvs_open("nvs.net80211", NVS_READONLY, &h) == ESP_OK) {
        uint8_t buf[64] = {0};
        size_t len = sizeof(buf);
        esp_err_t err = nvs_get_blob(h, "sta.ssid", buf, &len);
        nvs_close(h);

        if (err == ESP_OK && len >= 1) {
            uint8_t first = buf[0];

            // Format A: [len][ssid...]
            if (first > 0 && first <= 32 && (size_t)(first + 1) <= len) {
                char ssid[33] = {0};
                memcpy(ssid, buf + 1, first);
                DEBUG_PRINT("Legacy NVS format A, SSID: ");
                DEBUG_PRINTLN(ssid);
                setFlag(true);   // promote to our flag
                return true;
            }

            // Format B: plain null-terminated string
            if (first != 0 && first != 0xFF) {
                DEBUG_PRINT("Legacy NVS format B, SSID: ");
                DEBUG_PRINTLN((const char*)buf);
                setFlag(true);
                return true;
            }
        }
    }

    DEBUG_PRINTLN("No credentials found anywhere");
    return false;
}

// -----------------------------------------------------------------
// Boot-time check
// -----------------------------------------------------------------

void NetworkManager::ensureWiFiConfigured() {
    // Check BEFORE initializing the WiFi driver
    bool hasCreds = wifiCredentialsPresent();

    WiFi.mode(WIFI_STA);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (hasCreds) {
        DEBUG_PRINTLN("WiFi config present — proceeding normally");
        return;
    }

    DEBUG_PRINTLN("No WiFi config — entering AP portal");
    LEDController::setState(LedState::WIFI_AP_MODE);

    WiFiManager wm;
    wm.setConfigPortalTimeout(AP_PORTAL_TIMEOUT_SEC);
    wm.setBreakAfterConfig(true);

    bool ok = wm.startConfigPortal(AP_PORTAL_SSID);

    if (ok && WiFi.status() == WL_CONNECTED) {
        DEBUG_PRINTLN("WiFi configured successfully");
        setFlag(true);                 // <-- our flag, guaranteed to persist
        LEDController::blinkColor(CRGB::Green, 2, 300);
        vTaskDelay(pdMS_TO_TICKS(700));
    } else {
        DEBUG_PRINTLN("Config portal ended without success");
    }

    // Force WiFi driver to flush creds to NVS (best effort)
    WiFi.persistent(true);
    WiFi.begin();
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP.restart();
}

// -----------------------------------------------------------------
// Init & task
// -----------------------------------------------------------------

void NetworkManager::init() {
    WiFi.mode(WIFI_STA);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    mqttClient.setSocketTimeout(5);
    randomSeed(esp_random());
}

bool NetworkManager::isWiFiConnected() { return WiFi.status() == WL_CONNECTED; }
bool NetworkManager::isMQTTConnected() { return mqttClient.connected(); }

// -----------------------------------------------------------------
// WiFi reconnect
// -----------------------------------------------------------------

void NetworkManager::reconnectWiFi() {
    static int attempt = 0;
    static int wait = 0;
    static int cycle = MAX_WIFI_ATTEMPTS;

    if (WiFi.status() == WL_CONNECTED) {
        attempt = 0; wait = 0; cycle = MAX_WIFI_ATTEMPTS;
        return;
    }

    if (attempt < WIFI_ATTEMPT_COUNT) {
        DEBUG_PRINTLN("WiFi connect attempt...");
        WiFi.begin();
        attempt++;

        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED &&
               (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (WiFi.status() == WL_CONNECTED) {
            DEBUG_PRINTLN("WiFi connected");
            attempt = 0; wait = 0; cycle = MAX_WIFI_ATTEMPTS;
        }
    } else if (wait < WIFI_WAIT_COUNT) {
        wait++;
        vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_DELAY_MS));
    } else {
        attempt = 0; wait = 0; cycle--;
        if (cycle <= 0) {
            DEBUG_PRINTLN("Max WiFi cycles reached, restarting");
            ESP.restart();
        }
    }
}

// -----------------------------------------------------------------
// MQTT reconnect
// -----------------------------------------------------------------

void NetworkManager::reconnectMQTT() {
    static int attempt = 0;
    static char clientId[28] = {0};

    if (mqttClient.connected()) { attempt = 0; return; }

    if (attempt >= MQTT_ATTEMPT_COUNT) {
        DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting");
        ESP.restart();
    }

    if (clientId[0] == '\0') {
        snprintf(clientId, sizeof(clientId), "dma_pgb_%08X",
                 (unsigned)esp_random());
    }

    DEBUG_PRINTLN("MQTT connect attempt...");

    const char* willTopic = MQTT_PUB_TOPIC;
    const char* willMsg   = "offline";
    uint8_t     willQos   = 0;
    bool        willRetain = false;

    bool ok = mqttClient.connect(
        clientId,
        MQTT_USER, MQTT_PASSWORD,
        willTopic, willQos, willRetain, willMsg
    );

    if (ok) {
        DEBUG_PRINTLN("MQTT connected");
        attempt = 0;

        char subTopic[MQTT_TOPIC_MAX];
        snprintf(subTopic, sizeof(subTopic), "%s/%s",
                 MQTT_SUB_TOPIC, getDeviceId());
        mqttClient.subscribe(subTopic);
        LEDController::blinkColor(CRGB::Green, 2, 300);
    } else {
        DEBUG_PRINT("MQTT failed, rc=");
        DEBUG_PRINTLN(mqttClient.state());
        attempt++;
        vTaskDelay(pdMS_TO_TICKS(MQTT_ATTEMPT_DELAY_MS));
    }
}

// -----------------------------------------------------------------
// Callback + queue
// -----------------------------------------------------------------

void NetworkManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
    char message[128];
    unsigned int n = (length < sizeof(message) - 1) ? length : sizeof(message) - 1;
    memcpy(message, payload, n);
    message[n] = '\0';

    DEBUG_PRINT("MQTT msg: ");
    DEBUG_PRINTLN(message);

    if (strcmp(message, "ping") == 0) {
        char pingPayload[MQTT_PAYLOAD_MAX];
        snprintf(pingPayload, sizeof(pingPayload), "%s,%s,%s,%d,%lu",
                 getDeviceId(),
                 WiFi.SSID().c_str(),
                 WiFi.localIP().toString().c_str(),
                 WiFi.RSSI(),
                 (unsigned long)HB_INTERVAL_MS);
        MQTTQueue::enqueue(MQTT_PUB_TOPIC, pingPayload);
    } else if (strcmp(message, "update_firmware") == 0) {
        if (otaTaskHandle == NULL) {
            xTaskCreatePinnedToCore(otaTask, "OTA", 8192, NULL, 1, &otaTaskHandle, 1);
        } else {
            DEBUG_PRINTLN("OTA task already running");
        }
    }
}

void NetworkManager::processMQTTQueue() {
    MqttMessage msg;
    if (MQTTQueue::dequeue(&msg, 0)) {
        if (mqttClient.publish(msg.topic, msg.payload)) {
            DEBUG_PRINT("Published: ");
            DEBUG_PRINTLN(msg.payload);
            LEDController::blinkColor(CRGB::Green, 1, 500);
        } else {
            DEBUG_PRINTLN("MQTT publish failed");
        }
    }
}

void NetworkManager::publish(const char* topic, const char* payload) {
    MQTTQueue::enqueue(topic, payload);
}

// -----------------------------------------------------------------
// Main task
// -----------------------------------------------------------------

void NetworkManager::task(void *param) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mqttClient.connected()) {
                LEDController::setState(LedState::MQTT_OFFLINE);
                reconnectMQTT();
            } else {
                LEDController::setState(LedState::OFF);
                mqttClient.loop();
                processMQTTQueue();
            }
        } else {
            LEDController::setState(LedState::WIFI_DISCONNECTED);
            reconnectWiFi();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}