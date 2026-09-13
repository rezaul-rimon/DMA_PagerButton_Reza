#ifndef RF_RECEIVER_H
#define RF_RECEIVER_H

#include <RCSwitch.h>
#include <map>
#include "config.h"

class RFReceiver {
public:
    static void init();
    static void task(void *param);
private:
    static RCSwitch mySwitch;
    static std::map<unsigned long, unsigned long> lastSeenMap;
    static unsigned long lastGlobalTime;
    static bool isDuplicate(unsigned long code, unsigned long now);
};

#endif