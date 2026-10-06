#ifndef RF_RECEIVER_H
#define RF_RECEIVER_H

#include <RCSwitch.h>
#include "config.h"

class RFReceiver {
public:
    static void init();
    static void task(void *param);

private:
    struct Entry {
        unsigned long code;
        unsigned long time;
    };

    static RCSwitch mySwitch;
    static Entry history[RF_HISTORY_SIZE];
    static int historyIdx;
    static unsigned long lastGlobalTime;
    static bool isDuplicate(unsigned long code, unsigned long now);
};

#endif