#include <Arduino.h>
// xTaskCreatePinnedToCore(SEND, "Core 1 SEND", 8192, NULL, 3, T_SEND, 1);
TaskHandle_t *T_SEND;
static void CommTask(void *pvParameter) {

  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 10);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("TASK_SEND","Time Out!!!");
    }
  }
}