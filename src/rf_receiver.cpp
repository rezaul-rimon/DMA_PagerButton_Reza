#include "rf_receiver.h"
#include "main.h"
#include "led_controller.h"
#include "mqtt_queue.h"

RCSwitch RFReceiver::mySwitch;
std::map<unsigned long, unsigned long> RFReceiver::lastSeenMap;
unsigned long RFReceiver::lastGlobalTime = 0;

void RFReceiver::init() {
    mySwitch.enableReceive(digitalPinToInterrupt(RF_PIN));
}

bool RFReceiver::isDuplicate(unsigned long code, unsigned long now) {
    if (now - lastGlobalTime < GLOBAL_DEBOUNCE_MS) return true;
    auto it = lastSeenMap.find(code);
    if (it != lastSeenMap.end() && (now - it->second < SENSOR_DEBOUNCE_MS)) {
        return true;
    }
    return false;
}

void RFReceiver::task(void *param) {
    for (;;) {
        if (mySwitch.available()) {
            unsigned long receivedCode = mySwitch.getReceivedValue();
            int bitLength = mySwitch.getReceivedBitlength();
            unsigned long now = millis();

            if (bitLength >= 24 && !isDuplicate(receivedCode, now)) {
                lastSeenMap[receivedCode] = now;
                lastGlobalTime = now;

                DEBUG_PRINTLN(String("RF Received: ") + String(receivedCode) + " (" + String(bitLength) + " bits)");

                LEDController::blinkColor(CRGB::Blue, 1, 250);

                uint32_t code = (uint32_t)receivedCode;
                if (xQueueSend(rfQueue, &code, 0) != pdTRUE) {
                    DEBUG_PRINTLN("RF queue full, dropping signal");
                }
            }
            mySwitch.resetAvailable();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}