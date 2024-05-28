/******************************************************************************
 *  Filename:    RTC_Alarm.h
 *
 *  Description: RTC_Alarm
 *
 *  Created:     2024-03-16
 *  Author:      RIVER
 *
 *  Modification History:
 *  - 2024-03-16: Created the file.
 *  - ...
 *****************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include <ctime>

class RTC_Alarm : public RTC_DS3231 {
private:
    byte charging = 15;
    byte Chg_end = 15;
public:
    bool init(TwoWire *wire){
        return begin(wire);
    }
    void update(){
        // RTC_DS3231::update();
    }
    void FirstBoot(){
        // 清除并禁用闹钟
        clearAlarm(1); 
        disableAlarm(1); 
        // 手动设置当前时间和闹钟时间
        adjust(DateTime(F(__DATE__),F(__TIME__)));
        DateTime time = now();
    #ifdef TEST_RTC
        DateTime alarm_time(24,4,17,time.hour(),time.minute() + 3,01); 
    #else
        DateTime alarm_time(24,4,17,9,05,01); 
    #endif
        // 设置并启用闹钟时间
        if(setAlarm1(alarm_time,DS3231_A1_Hour) == false){
            Serial.printf("[ERROR]setAlarm1!!!\n"); 
        }
    }

    void print_wakeup_reason(){
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason)
    {
        case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;//由外部信号使用RTC_IO引起的唤醒
        case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;//使用RTC_CNTL的外部信号引起的唤醒
        case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;//由计时器引起的唤醒
        case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;//由触摸板引起的唤醒
        case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;//由触摸板引起的唤醒
        default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }
    }
void convertToBeijingTimeOnly(struct tm *utc) {

    // 东八区与UTC的小时差
    int offset_hours = 8;

    // 添加时间差
    utc->tm_hour += offset_hours;

    // 检查是否需要跨天，这里我们忽略日期
    if (utc->tm_hour >= 24) {
      utc->tm_hour -= 24;
    } else if (utc->tm_hour < 0) {
      utc->tm_hour += 24;
    }
  }

};

