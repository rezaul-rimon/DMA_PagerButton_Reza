#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

extern TaskHandle_t networkTaskHandle;
extern TaskHandle_t mainTaskHandle;
extern TaskHandle_t wifiResetTaskHandle;
extern TaskHandle_t otaTaskHandle;
extern TaskHandle_t rfTaskHandle;
extern TaskHandle_t ledTaskHandle;

extern QueueHandle_t rfQueue;
extern const char* DEVICE_ID;

void setup();
void loop();
void networkTask(void *param);
void mainTask(void *param);
void wifiResetTask(void *param);
void rfTask(void *param);
void otaTask(void *param);

#endif