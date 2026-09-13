#ifndef DEVICE_ID_H
#define DEVICE_ID_H

#include <Preferences.h>

class DeviceID {
public:
    static void init();
    static const char* get();
private:
    static Preferences prefs;
    static String id;
};

#endif