#include "device_id.h"
#include "config.h"

Preferences DeviceID::prefs;
String DeviceID::id;

void DeviceID::init() {
    prefs.begin("device_data", false);

    #if CHANGE_DEVICE_ID
        id = String(WORK_PACKAGE) + GW_TYPE + FIRMWARE_UPDATE_DATE + DEVICE_SERIAL;
        prefs.putString("device_id", id);
        Serial.println("Device ID updated in Preferences: " + id);
    #else
        id = prefs.getString("device_id", "UNKNOWN");
        Serial.println("Restored Device ID from Preferences: " + id);
    #endif

    prefs.end();
}

const char* DeviceID::get() {
    return id.c_str();
}