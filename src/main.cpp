#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>  // WiFiManager library
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <RCSwitch.h>
#include <Preferences.h>
RCSwitch mySwitch = RCSwitch();

#include <map>
std::map<unsigned long, unsigned long> lastRFReceivedTimeMap;
unsigned long lastRFGlobalReceivedTime = 0;  // Global debounce

// Configuration Section
// #define Fast_LED false
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }

//First time is must set to true.
#define CHANGE_DEVICE_ID 0

#if CHANGE_DEVICE_ID
#define WORK_PACKAGE "1225"
#define GW_TYPE "01" //For Pager Button
#define FIRMWARE_UPDATE_DATE "250304" 
#define DEVICE_SERIAL "0001"
// #define DEVICE_ID WORK_PACKAGE GW_TYPE FIRMWARE_UPDATE_DATE DEVICE_SERIAL
#endif

const char* DEVICE_ID;

Preferences preferences;

#define HB_INTERVAL 5*60*1000
// #define DATA_INTERVAL 15*1000

// WiFi and MQTT reconnection time config
#define WIFI_ATTEMPT_COUNT 60
#define WIFI_ATTEMPT_DELAY 1000
#define WIFI_WAIT_COUNT 60
#define WIFI_WAIT_DELAY 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 10
#define MQTT_ATTEMPT_DELAY 5000

int wifiAttemptCount = WIFI_ATTEMPT_COUNT;
int wifiWaitCount = WIFI_WAIT_COUNT;
int maxWifiAttempts = MAX_WIFI_ATTEMPTS;
int mqttAttemptCount = MQTT_ATTEMPT_COUNT;

const char* mqtt_server = "broker2.dma-bd.com";
const char* mqtt_user = "broker2";
const char* mqtt_password = "Secret!@#$1234";
const char* mqtt_hb_topic = "DMA/PagerButton/HB";
const char* mqtt_pub_topic = "DMA/PagerButton/PUB";
const char* mqtt_sub_topic = "DMA/PagerButton/SUB";
const char* ota_url = "https://raw.githubusercontent.com/rezaul-rimon/DMA_PagerButton_Reza/main/ota/firmware.bin";

void performOTA();

#define LED_PIN 21
#define RF_PIN 25

WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);

TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t wifiResetTaskHandle;

#define WIFI_RESET_BUTTON_PIN 0
bool wifiResetFlag = false;

// Function to reconnect to WiFi
void reconnectWiFi() {
  digitalWrite(LED_PIN, HIGH);
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting WiFi connection...");
      WiFi.begin();  // Use saved credentials
      wifiAttemptCount--;
      DEBUG_PRINTLN("Remaining WiFi attempts: " + String(wifiAttemptCount));
      vTaskDelay(pdMS_TO_TICKS(WIFI_ATTEMPT_DELAY));
    } else if (wifiWaitCount > 0) {
      wifiWaitCount--;
      DEBUG_PRINTLN("WiFi wait... retrying in a moment");
      DEBUG_PRINTLN("Remaining WiFi wait time: " + String(wifiWaitCount) + " seconds");
      vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_DELAY));
    } else {
      wifiAttemptCount = WIFI_ATTEMPT_COUNT;
      wifiWaitCount = WIFI_WAIT_COUNT;
      maxWifiAttempts--;
      if (maxWifiAttempts <= 0) {
        DEBUG_PRINTLN("Max WiFi attempt cycles exceeded, restarting...");
        ESP.restart();
      }
    }
  }
}


void reconnectMQTT() {
  digitalWrite(LED_PIN, HIGH);
  if (!client.connected()) {
    char clientId[24];
    snprintf(clientId, sizeof(clientId), "dma_pgb_%04X%04X%04X", random(0xffff), random(0xffff), random(0xffff));

    if (mqttAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting MQTT connection...");
      if (client.connect(clientId, mqtt_user, mqtt_password)) {
        DEBUG_PRINTLN("MQTT connected");
        DEBUG_PRINT("Client_ID: ");
        DEBUG_PRINTLN(clientId);
        digitalWrite(LED_PIN, LOW);

        char topic[48];
        snprintf(topic, sizeof(topic), "%s/%s", mqtt_sub_topic, DEVICE_ID);
        client.subscribe(topic);
      } else {
        DEBUG_PRINTLN("MQTT connection failed");
        mqttAttemptCount--;
        vTaskDelay(pdMS_TO_TICKS(MQTT_ATTEMPT_DELAY));
      }
    } else {
      DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting...");
      ESP.restart();
    }
  }
}

// MQTT Callback Start
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  DEBUG_PRINTLN("Message arrived on topic: " + String(topic));
  DEBUG_PRINTLN("Message content: " + message); 

  // Check if the message is "get_from_sd_card"
  if (message == "update_firmware") {
    DEBUG_PRINTLN("Trigger performOTA()...");
   performOTA();
  }
}
// MQTT Callback End

// Start Perform OTA
void performOTA() {
  Serial.println("Starting OTA update...");

  HTTPClient http;
  http.begin(ota_url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    Serial.printf("Content-Length: %d bytes\n", contentLength);
    if (Update.begin(contentLength)) {
      Update.writeStream(http.getStream());
      if (Update.end() && Update.isFinished()) {
        Serial.println("OTA update completed. Restarting...");
        client.loop(); // Ensure MQTT client processes the publish
        client.publish(mqtt_pub_topic, "OTA update successful");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to allow message to send
        ESP.restart();
      } else {
        Serial.println("OTA update failed!");
        client.publish(mqtt_pub_topic, "OTA update failed, restarting with last firmware");
      }
    } else {
      Serial.println("OTA begin failed!");
      client.publish(mqtt_pub_topic, "OTA begin failed, restarting with last firmware");
    }
  } else {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    client.publish(mqtt_pub_topic, "OTA HTTP request failed, restarting with last firmware");
  }
  http.end();

  vTaskDelay(1000 / portTICK_PERIOD_MS); // Give time for MQTT message to send
  ESP.restart();
}


void networkTask(void *param) {
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnectMQTT();
      }
    } else {
      reconnectWiFi();
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void wifiResetTask(void *param) {
  for (;;) {
    if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
      unsigned long pressStartTime = millis();
      DEBUG_PRINTLN("Button Pressed....");

      while (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
        if (millis() - pressStartTime >= 5000) {
          DEBUG_PRINTLN("5 seconds holding time reached, starting WiFiManager...");
          vTaskSuspend(networkTaskHandle);
          vTaskSuspend(mainTaskHandle);
          wm.resetSettings();
          wm.autoConnect("DMA_Pager_Button"); // Start AP without reboot
          ESP.restart();  // Optional: restart after setup
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    } else {
      //
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/*
void mainTask(void *param) {
  for (;;) {
    static unsigned long last_hb_send_time = 0;
    unsigned long now = millis();

    // **Send Heartbeat Every HB_INTERVAL**
    if (now - last_hb_send_time >= HB_INTERVAL) {
      last_hb_send_time = now;
      if (client.connected()) {
        char hb_data[50];
        snprintf(hb_data, sizeof(hb_data), "%s,wifi_connected", DEVICE_ID);
        client.publish(mqtt_hb_topic, hb_data);
        DEBUG_PRINTLN("Heartbeat sent Successfully");

        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_PIN, LOW);
      } else {
        DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
      }
    }

    // **Handle RF Signal Reception with Dual-Level Debounce**
    if (mySwitch.available()) {
      unsigned long receivedCode = mySwitch.getReceivedValue();

      // **Short-Term Global Debounce (Ignore if received within 200ms)**
      if (now - lastRFGlobalReceivedTime < 100) {
        mySwitch.resetAvailable();
        return;
      }

      // **Per-Sensor Debounce (Ignore same sensor within 2 sec)**
      if (lastRFReceivedTimeMap.find(receivedCode) == lastRFReceivedTimeMap.end() || 
          (now - lastRFReceivedTimeMap[receivedCode] > 2000)) {  

        lastRFReceivedTimeMap[receivedCode] = now;  // Update per-sensor time
        lastRFGlobalReceivedTime = now;  // Update global debounce

        // **Debug Output**
        DEBUG_PRINTLN(String("RF Received: ") + String(receivedCode));
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_PIN, LOW);

        // **Send Data to MQTT**
        char data[50];
        snprintf(data, sizeof(data), "%s,%lu", DEVICE_ID, receivedCode);
        client.publish(mqtt_pub_topic, data);
        DEBUG_PRINTLN(String("Data Sent to MQTT: ") + String(data));

        // **Indication Blink**
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(50));
          digitalWrite(LED_PIN, LOW);
          vTaskDelay(pdMS_TO_TICKS(50));
        }
      }

      mySwitch.resetAvailable();
    }

    vTaskDelay(1); // Keep FreeRTOS responsive
  }
}
*/

void mainTask(void *param) {
  for (;;) {
    static unsigned long last_hb_send_time = 0;
    unsigned long now = millis();

    // **Send Heartbeat Every HB_INTERVAL**
    if (now - last_hb_send_time >= HB_INTERVAL) {
      last_hb_send_time = now;
      if (client.connected()) {
        char hb_data[50];
        snprintf(hb_data, sizeof(hb_data), "%s,wifi_connected", DEVICE_ID);
        client.publish(mqtt_hb_topic, hb_data);
        DEBUG_PRINTLN("Heartbeat sent Successfully");

        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_PIN, LOW);
      } else {
        DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
      }
    }

    // **Handle RF Signal Reception with Bit Length Filtering**
    if (mySwitch.available()) {
      unsigned long receivedCode = mySwitch.getReceivedValue();
      int bitLength = mySwitch.getReceivedBitlength(); // Get bit length of the received signal

      // **Ignore signals that do not match the expected bit length (e.g., < 24 bits)**
      if (bitLength < 24) {  
        DEBUG_PRINTLN(String("Ignored RF Signal: ") + String(receivedCode) + " (Bits: " + String(bitLength) + ")");
        mySwitch.resetAvailable();
        continue;
      }

      // **Short-Term Global Debounce (Ignore if received within 100ms)**
      if (now - lastRFGlobalReceivedTime < 100) {
        mySwitch.resetAvailable();
        continue;
      }

      // **Per-Sensor Debounce (Ignore same sensor within 2 sec)**
      if (lastRFReceivedTimeMap.find(receivedCode) == lastRFReceivedTimeMap.end() || 
          (now - lastRFReceivedTimeMap[receivedCode] > 2000)) {  

        lastRFReceivedTimeMap[receivedCode] = now;  // Update per-sensor time
        lastRFGlobalReceivedTime = now;  // Update global debounce

        // **Debug Output**
        DEBUG_PRINTLN(String("Valid RF Received: ") + String(receivedCode) + " (Bits: " + String(bitLength) + ")");
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_PIN, LOW);

        // **Send Data to MQTT**
        char data[50];
        snprintf(data, sizeof(data), "%s,%lu", DEVICE_ID, receivedCode);
        client.publish(mqtt_pub_topic, data);
        DEBUG_PRINTLN(String("Data Sent to MQTT: ") + String(data));

        // **Indication Blink**
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(50));
          digitalWrite(LED_PIN, LOW);
          vTaskDelay(pdMS_TO_TICKS(50));
        }
      }

      mySwitch.resetAvailable();
    }

    vTaskDelay(1); // Keep FreeRTOS responsive
  }
}



void setup() {
  Serial.begin(115200);
  preferences.begin("device_data", false);  // Open Preferences (NVS)
  static String device_id; // Static variable to persist scope
  
  #if CHANGE_DEVICE_ID
    // Construct new device ID
    device_id = String(WORK_PACKAGE) + GW_TYPE + FIRMWARE_UPDATE_DATE + DEVICE_SERIAL;
    
    // Save device ID to Preferences
    preferences.putString("device_id", device_id);
    Serial.println("Device ID updated in Preferences: " + device_id);
  #else
    // Restore device ID from Preferences
    device_id = preferences.getString("device_id", "UNKNOWN");
    Serial.println("Restored Device ID from Preferences: " + device_id);
  #endif

  DEVICE_ID = device_id.c_str(); // Assign to global pointer

  preferences.end();

  DEBUG_PRINT("Device ID: ");
  DEBUG_PRINTLN(DEVICE_ID);

  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  mySwitch.enableReceive(digitalPinToInterrupt(RF_PIN));

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  // WiFiManager wm;
  // bool res = wm.autoConnect("DMA_Pager_Button");  // Start AP if WiFi is not configured

  // if (!res) {
  //   DEBUG_PRINTLN("Failed to connect, starting AP mode...");
  // } else {
  //   DEBUG_PRINTLN("Connected to WiFi!");
  // }

  xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, &mainTaskHandle, 1);
  xTaskCreatePinnedToCore(wifiResetTask, "WiFi Reset Task", 8*1024, NULL, 1, &wifiResetTaskHandle, 1);
}


void loop(){

}