#ifndef CONFIG_H
#define CONFIG_H

#define MQTT_SERVER "broker2.dma-bd.com"
#define MQTT_PORT 1883
#define MQTT_USER "broker2"
#define MQTT_PASSWORD "Secret!@#$1234"
#define MQTT_HB_TOPIC "DMA/PagerButton/HB"
#define MQTT_PUB_TOPIC "DMA/PagerButton/PUB"
#define MQTT_SUB_TOPIC "DMA/PagerButton/SUB"

#define OTA_URL "https://raw.githubusercontent.com/rezaul-rimon/DMA_PagerButton_Reza/with-ota/ota/firmware.bin"

#define CHANGE_DEVICE_ID 0
#if CHANGE_DEVICE_ID
  #define WORK_PACKAGE "1225"
  #define GW_TYPE "01"
  #define FIRMWARE_UPDATE_DATE "260901"
  #define DEVICE_SERIAL "0001"
#endif

#define HB_INTERVAL_MS 2 * 60 * 1000
#define WIFI_ATTEMPT_COUNT 60
#define WIFI_ATTEMPT_DELAY_MS 1000
#define WIFI_WAIT_COUNT 60
#define WIFI_WAIT_DELAY_MS 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 10
#define MQTT_ATTEMPT_DELAY_MS 5000

#define LED_PIN 4
#define RF_PIN 25
#define WIFI_RESET_BUTTON_PIN 21

#define NUM_LEDS 1
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

#define GLOBAL_DEBOUNCE_MS 20
#define SENSOR_DEBOUNCE_MS 2000

#define DEBUG_MODE true
#define DEBUG_PRINT(x)    if (DEBUG_MODE) Serial.print(x)
#define DEBUG_PRINTLN(x)  if (DEBUG_MODE) Serial.println(x)

#endif