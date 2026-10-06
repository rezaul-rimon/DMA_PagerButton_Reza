#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "config.h"

enum class LedState {
    OFF,
    WIFI_DISCONNECTED,   // solid red
    MQTT_OFFLINE,        // solid yellow
    WIFI_AP_MODE         // solid blue
};

struct BlinkEvent {
    CRGB color;
    uint16_t times;
    uint16_t delayMs;
};

class LEDController {
public:
    static void init();
    static void task(void *param);
    static void setState(LedState state);
    static void blinkColor(CRGB color, uint16_t times = 1, uint16_t delayMs = 100);

private:
    static CRGB leds[NUM_LEDS];
    static volatile LedState currentBaseState;
    static QueueHandle_t blinkQueue;
    static BlinkEvent currentBlink;
    static bool isBlinking;
    static uint16_t blinkCount;
    static bool blinkOn;
    static unsigned long lastBlinkTime;

    static void applyBaseColor(LedState state);
};

#endif