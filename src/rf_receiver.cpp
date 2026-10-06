#include "rf_receiver.h"
#include "main.h"
#include "led_controller.h"

RCSwitch RFReceiver::mySwitch;
RFReceiver::Entry RFReceiver::history[RF_HISTORY_SIZE] = {};
int RFReceiver::historyIdx = 0;
unsigned long RFReceiver::lastGlobalTime = 0;

void RFReceiver::init() {
    mySwitch.enableReceive(digitalPinToInterrupt(RF_PIN));
}

bool RFReceiver::isDuplicate(unsigned long code, unsigned long now) {
    if (now - lastGlobalTime < GLOBAL_DEBOUNCE_MS) return true;
    for (int i = 0; i < RF_HISTORY_SIZE; i++) {
        if (history[i].code == code &&
            (now - history[i].time) < SENSOR_DEBOUNCE_MS) {
            return true;
        }
    }
    return false;
}

void RFReceiver::task(void *param) {
    for (;;) {
        if (mySwitch.available()) {
            unsigned long code = mySwitch.getReceivedValue();
            int bitLength = mySwitch.getReceivedBitlength();
            unsigned long now = millis();

            if (bitLength >= 24 && !isDuplicate(code, now)) {
                history[historyIdx].code = code;
                history[historyIdx].time = now;
                historyIdx = (historyIdx + 1) % RF_HISTORY_SIZE;
                lastGlobalTime = now;

                DEBUG_PRINT("RF Received: ");
                DEBUG_PRINTLN(code);

                LEDController::blinkColor(CRGB::Blue, 1, 300);

                uint32_t c = (uint32_t)code;
                if (xQueueSend(rfQueue, &c, 0) != pdTRUE) {
                    DEBUG_PRINTLN("RF queue full, drop");
                }
            }
            mySwitch.resetAvailable();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}