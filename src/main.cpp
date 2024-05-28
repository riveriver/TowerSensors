#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <SoftwareSerial.h>
#include "BoardConfig.h"
#include "SLED.h"
#include "RTC_Alarm.h"
#include "GPS_UM982.h"
#include "IMU_HWT9073.h"
#include "TowerComm.h"

// TASK
unsigned long slow_sync = millis();
// RTC 
#define RTC_ADDR 0x75
#define RTC_WAKEUP_IO GPIO_NUM_7
TwoWire wire_0 = TwoWire(0);
RTC_Alarm rtc_alarm;
DateTime now_time;
DateTime sleep_time(24,5,7,17,00,00);
DateTime alarm_time(24,5,7,9,00,00);
uint8_t bat_send = 0;
ICACHE_RAM_ATTR void alarm_interrupt()
{
	Serial.println("alarm_interrupt !");
};
RTC_DATA_ATTR unsigned  int bootCount = 0;
RTC_DATA_ATTR unsigned  int bootFlag = false;
// SD
#define Version "3.0.0"
// IMU
#ifdef ONLY_IMU
#define IMU_ADDR 0x50
#else
#define IMU_ADDR 0x55
#endif
// GPS
GPS_UM982 gps;	
// BAT
#define BAT_ADC_IO GPIO_NUM_6
// COMM
ImuCommFrame imu_comm;
GpsCommFrame gps_comm;
// 
void InitSystem();
int8_t BatteryFSM();
void DeepSleep();
void InitRTC();
void RTC_FSM();
void IMU_FSM();
void GPS_FSM();
/**************************** Framework Begin *******************************************/
void setup()
{
	InitSystem();
	InitRTC();
	BatteryFSM();
	Imu_initHwt9073(IMU_ADDR);
#ifndef ONLY_IMU
		gps.init();
#endif
	InitRadio();
}

void loop(){
	// if(millis() - slow_sync > 5  * 1000){
	if(millis() - slow_sync > 5 * 60 * 1000){
		slow_sync = millis();
		bat_send = BatteryFSM();
		SendToUpper((uint8_t *)&bat_send, sizeof(bat_send), 0x03);
	}
	RTC_FSM();
	IMU_FSM();
	GPS_FSM();
}

void DeepSleep(){
	DateTime now = rtc_alarm.now();
	Serial.printf("auto_sleep_time:%02d:%02d:%02d\n",
	now.hour(), now.minute(), now.second());
	delay(100);
	digitalWrite(34, HIGH);
	digitalWrite(33, LOW);
	delay(100);
	esp_deep_sleep_start();
}
void pack_gps_comm_frame()
{
	now_time = rtc_alarm.now();
	gps_comm.time.hour = now_time.hour();
	gps_comm.time.minute = now_time.minute();
	gps_comm.time.second = now_time.second();
	
	gps_comm.hdop = gps.HDOP;
	// gps_comm.sol_status = gps.sol;
	// gps_comm.satellites = gps.satellites;
	gps_comm.latitude = gps.latitude;
	// if (gps.lat == 'N')
	// {
	// 	gps_comm.lat_ns = 1;
	// }
	// else if (gps.lat == 'S')
	// {
	// 	gps_comm.lat_ns = 2;
	// }
	gps_comm.longitude = gps.longitude;
	// if (gps.lon == 'E')
	// {
	// 	gps_comm.lon_ew = 1;
	// }
	// else if (gps.lon == 'W')
	// {
	// 	gps_comm.lon_ew = 2;
	// }
	gps_comm.heading = gps.headling;
	gps_comm.pitch = gps.pitch;
	gps_comm.roll = gps.roll;
}

void InitSystem()
{
	Serial.begin(115200);
Serial.printf("VERSION:%s;BUILD_FLAGS:",Version);
#ifdef DEBUG_MODE
	Serial.printf("DEBUG_MODE,");
#else
	Serial.printf("COMMON_MODE,");
#endif
#ifdef LIVE_ALWAY
	Serial.printf("LIVE_ALWAY,");
#endif
#ifdef ONLY_IMU
	Serial.printf("ONLY_IMU,");
#endif
#ifdef TEST_IMU
	Serial.printf("TEST_IMU,");
#endif
#ifdef TEST_GPS
	Serial.printf("TEST_GPS,");
#endif
#ifdef TEST_RTC
	Serial.printf("TEST_RTC,");
#endif
	Serial.println("");
	// EN 18V
	pinMode(34, OUTPUT);
	digitalWrite(34, LOW);
	Serial.println("ENABLE 18V(IO34 LOW)");
	// EN 5V 
	pinMode(33, OUTPUT);
	digitalWrite(33, HIGH);
	Serial.println("ENABLE 5V (IO33 HIGH)");
    // EN_SD
	pinMode(14, OUTPUT);
	digitalWrite(14, HIGH);
	Serial.println("EN_SD(IO_14 HIGH)");
	// Bat
	pinMode(BAT_ADC_IO, INPUT);
	/* TowerBoard-v2.0 end */

}

void InitRTC(){
	if (wire_0.begin(8, 9, 400000U) == false)
	{
		Serial.println("[ERROR]wire_0 begin fail!");
	}
	if (rtc_alarm.init(&wire_0) == false)
	{
		Serial.println("[ERROR]rtc begin fail!");
	}
	interrupts();
	attachInterrupt(digitalPinToInterrupt(RTC_WAKEUP_IO), alarm_interrupt, FALLING);
	pinMode(RTC_WAKEUP_IO, INPUT_PULLUP);
	esp_sleep_enable_ext0_wakeup(RTC_WAKEUP_IO, 0);
    // 从网络时间服务器上获取并设置时间,获取成功后芯片会使用RTC时钟保持时间的更新(need wifi)
	// const char *ntpServer = "pool.ntp.org";
	// const long gmtOffset_sec = 8 * 3600;
	// const int daylightOffset_sec = 0;
    // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
	// 
	// if (++bootCount != 1)
	// {
	rtc_alarm.print_wakeup_reason();
	// }
	// else{
	// 	rtc_alarm.adjust(DateTime(F(__DATE__),F(__TIME__)));
	// }
	now_time = rtc_alarm.now();
	rtc_alarm.clearAlarm(1); 
	rtc_alarm.disableAlarm(1);
#ifdef TEST_RTC
	DateTime alarm_time(24,4,17,now_time.hour(),now_time.minute() + 2,01); 
#endif
	if(rtc_alarm.setAlarm1(alarm_time,DS3231_A1_Hour) == false){
		Serial.printf("[ERROR]setAlarm1 fail!\n"); 
	}  


/* Serial_Log */
	Serial.printf("boot count:%d\n", bootCount);
	Serial.printf("wakeup_time:%02d:%02d:%02d\n",
				  now_time.hour(), now_time.minute(), now_time.second());
	alarm_time = rtc_alarm.getAlarm1();
	Serial.printf("next_alarm_time:%02d:%02d:%02d\n",
			alarm_time.hour(), alarm_time.minute(), alarm_time.second());
	Serial.printf("next_sleep_time:%02d:%02d:%02d\n",
			sleep_time.hour(), sleep_time.minute(), sleep_time.second());
}

void RTC_FSM(){
#ifdef TEST_RTC
	if (bootFlag == false)
	{
		bootFlag = true;
		Serial.printf("[AutoSleepReason]RtcTest\n");
		DeepSleep();
	}
#else
#ifndef LIVE_ALWAY
	now_time = rtc_alarm.now();
	if (now_time.hour() >= sleep_time.hour())
	{ 
		Serial.printf("[AutoSleepReason]ReachSleepTime\n");
		DeepSleep();
	}
#endif 
#endif
}

void IMU_FSM(){
#ifdef TEST_IMU
   test_raw_send_receive();
#else
	test_receive_parse(IMU_ADDR);
#endif
    now_time = rtc_alarm.now();
	imu_comm.time.hour = now_time.hour();
	imu_comm.time.minute = now_time.minute();
	imu_comm.time.second = now_time.second();
	for (int i = 0; i < 4; i++)
	{
		imu_comm.quat[i] = quat_raw[i];
	}
	SendToUpper((uint8_t *)&imu_comm, sizeof(imu_comm), 0x02);
	for (int i = 0; i < 4; i++)
	{
		imu_comm.quat[i] = quat_raw[i];
	}
	SendToUpper((uint8_t *)&imu_comm, sizeof(imu_comm), 0x02);
#ifdef DEBUG_MODE
 	Serial.printf("0X%X:%f,%f,%f,%f\n\r",IMU_ADDR,quat_raw[0],quat_raw[1],quat_raw[2],quat_raw[3]);
#endif
}
	
void GPS_FSM(){
// Serial.printf("GPS_START:%d\n",millis());
#ifndef ONLY_IMU
#ifdef TEST_GPS
	gps.test_Parse();
#else
	gps.update();
#endif
// if (gps.latitude != 0 && gps.longitude != 0)
// {
	// Serial_Log.println(gps.OutputGpsInfo());
	pack_gps_comm_frame();
	SendToUpper((uint8_t *)&gps_comm, sizeof(gps_comm), 0x01);
// }else{
//     Serial.println("GPS data Invaild");
// }
#endif
}

int8_t BatteryFSM(){
	int sample = analogRead(BAT_ADC_IO);
	int8_t bat_per = (sample - 1670) * 100 / 550 ;
#ifdef DEBUG_MODE
	  Serial.printf("[Bat]bat_raw:%d;bat_per:%d\n",sample,bat_per);
#endif
	if(bat_per < 0){
		bat_per = 0;
		Serial.printf("[AutoSleepReason]LowBattery:%d\n",sample);
		DeepSleep();
	}
	if(bat_per > 100)bat_per = 100;
	return bat_per;
}
