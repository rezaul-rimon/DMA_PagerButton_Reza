#include "device_id.h"
#include "config.h"
#include <Preferences.h>

static char g_deviceId[DEVICE_ID_MAX_LEN] = {0};
static Preferences prefs;

void initDeviceId() {
    prefs.begin("device_data", false);

    #if CHANGE_DEVICE_ID
        snprintf(g_deviceId, sizeof(g_deviceId), "%s%s%s%s",
                 WORK_PACKAGE, GW_TYPE, FIRMWARE_UPDATE_DATE, DEVICE_SERIAL);
        prefs.putString("device_id", g_deviceId);
        Serial.print("Device ID updated: ");
        Serial.println(g_deviceId);
    #else
        // One-time String use at boot is acceptable
        String stored = prefs.getString("device_id", "UNKNOWN");
        strncpy(g_deviceId, stored.c_str(), sizeof(g_deviceId) - 1);
        g_deviceId[sizeof(g_deviceId) - 1] = '\0';
        Serial.print("Restored Device ID: ");
        Serial.println(g_deviceId);
    #endif

    prefs.end();
}

const char* getDeviceId() {
    return g_deviceId;
}