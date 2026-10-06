#include "led_controller.h"

CRGB LEDController::leds[NUM_LEDS];
volatile LedState LEDController::currentBaseState = LedState::OFF;
QueueHandle_t LEDController::blinkQueue = nullptr;
BlinkEvent LEDController::currentBlink;
bool LEDController::isBlinking = false;
uint16_t LEDController::blinkCount = 0;
bool LEDController::blinkOn = false;
unsigned long LEDController::lastBlinkTime = 0;

void LEDController::init() {
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    FastLED.clear();
    FastLED.show();
    blinkQueue = xQueueCreate(10, sizeof(BlinkEvent));
}

void LEDController::task(void *param) {
    LedState drawnBase = LedState::OFF;
    bool baseValid = false;

    for (;;) {
        if (isBlinking) {
            unsigned long now = millis();
            if (now - lastBlinkTime >= currentBlink.delayMs) {
                lastBlinkTime = now;
                blinkOn = !blinkOn;

                if (blinkOn) {
                    fill_solid(leds, NUM_LEDS, currentBlink.color);
                } else {
                    fill_solid(leds, NUM_LEDS, CRGB::Black);
                }
                FastLED.show();

                if (!blinkOn) {
                    blinkCount++;
                    if (blinkCount >= currentBlink.times) {
                        isBlinking = false;
                        baseValid = false;   // force redraw of base color
                    }
                }
            }
        } else {
            if (xQueueReceive(blinkQueue, &currentBlink, 0) == pdTRUE) {
                isBlinking = true;
                blinkCount = 0;
                blinkOn = false;
                lastBlinkTime = millis();
            } else if (!baseValid || drawnBase != currentBaseState) {
                LedState s = currentBaseState;
                applyBaseColor(s);
                drawnBase = s;
                baseValid = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void LEDController::setState(LedState state) {
    currentBaseState = state;
}

void LEDController::blinkColor(CRGB color, uint16_t times, uint16_t delayMs) {
    if (blinkQueue == nullptr) return;
    BlinkEvent event;
    event.color = color;
    event.times = times;
    event.delayMs = delayMs;
    xQueueSend(blinkQueue, &event, 0);
}

void LEDController::applyBaseColor(LedState state) {
    switch (state) {
        case LedState::OFF:               fill_solid(leds, NUM_LEDS, CRGB::Black);  break;
        case LedState::WIFI_DISCONNECTED: fill_solid(leds, NUM_LEDS, CRGB::Red);    break;
        case LedState::MQTT_OFFLINE:      fill_solid(leds, NUM_LEDS, CRGB::Yellow); break;
        case LedState::WIFI_AP_MODE:      fill_solid(leds, NUM_LEDS, CRGB::Blue);   break;
    }
    FastLED.show();
}