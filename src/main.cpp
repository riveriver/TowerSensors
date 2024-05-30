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

// IMU
#ifdef ONLY_IMU
#define IMU_ADDR 0x50
#else
#define IMU_ADDR 0x55
#endif
// GPS
GPS_UM982 gps;	
// COMM
ImuCommFrame imu_comm;
GpsCommFrame gps_comm;
// Function
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
	Imu_initHwt9073(IMU_ADDR);
#ifndef ONLY_IMU
		gps.init();
#endif
	InitRadio();
	pinMode(RTC_RESET_IO, INPUT_PULLUP);
}

void loop(){
	if(millis() - slow_sync > 5  * 1000){
	// if(millis() - slow_sync > 5 * 60 * 1000){
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
	Serial.printf("[Time]auto_sleep:%02d:%02d:%02d\n",
	now.hour(), now.minute(), now.second());
	delay(100);
	digitalWrite(P_MOS_IO, HIGH);
	digitalWrite(EN_IO, LOW);
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
	pinMode(P_MOS_IO, OUTPUT);
	digitalWrite(P_MOS_IO, LOW);
	Serial.println("[OnOff]ENABLE P_MOS");
	// EN 5V 
	pinMode(EN_IO, OUTPUT);
	digitalWrite(EN_IO, HIGH);
	Serial.println("[OnOff]ENABLE 5V");
    // EN_SD
	pinMode(14, OUTPUT);
	digitalWrite(14, HIGH);
	Serial.println("[OnOff]ENABLE SD");
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
	rtc_alarm.print_wakeup_reason();
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
	Serial.printf("[OnOff]boot_count:%d\n", bootCount);
	Serial.printf("[Time]boot:%02d:%02d:%02d\n",
				  now_time.hour(), now_time.minute(), now_time.second());
	alarm_time = rtc_alarm.getAlarm1();
	Serial.printf("[Time]next_alarm:%02d:%02d:%02d\n",
			alarm_time.hour(), alarm_time.minute(), alarm_time.second());
	Serial.printf("[Time]next_sleep:%02d:%02d:%02d\n",
			sleep_time.hour(), sleep_time.minute(), sleep_time.second());
}

void RTC_FSM(){
	if(digitalRead(GPIO_NUM_0) == LOW){
		rtc_alarm.adjust(DateTime(F(__DATE__),F(__TIME__)));
	}
	now_time = rtc_alarm.now();
	Serial.println();
	Serial.printf("[Time]now:%02d:%02d:%02d\n",
				  now_time.hour(), now_time.minute(), now_time.second());
#ifdef TEST_RTC
	if (bootFlag == false)
	{
		bootFlag = true;
		Serial.printf("[AutoSleep]RtcTest\n");
		DeepSleep();
	}
#else
#ifndef LIVE_ALWAY
	if (now_time.hour() >= sleep_time.hour())
	{ 
		Serial.printf("[AutoSleep]ReachSleepTime\n");
		DeepSleep();
	}
#endif 
#endif
}

void IMU_FSM(){
#ifdef TEST_IMU
   test_raw_send_receive();
	for (int i = 0; i < 4; i++)
	{
		imu_comm.quat[i] = 0.5;
	}
#else
	test_receive_parse(IMU_ADDR);
	for (int i = 0; i < 4; i++)
	{
		imu_comm.quat[i] = quat_raw[i];
	}
#endif
    now_time = rtc_alarm.now();
	imu_comm.time.hour = now_time.hour();
	imu_comm.time.minute = now_time.minute();
	imu_comm.time.second = now_time.second();
	SendToUpper((uint8_t *)&imu_comm, sizeof(imu_comm), 0x02);
	Serial.printf("[IMU]send:0X%X:%f,%f,%f,%f\n\r",IMU_ADDR,imu_comm.quat[0],imu_comm.quat[1],imu_comm.quat[2],imu_comm.quat[3]);
}
	
void GPS_FSM(){
#ifdef TEST_GPS
	gps.test_Parse();
	if (gps.latitude != 0 && gps.longitude != 0)
	{
	now_time = rtc_alarm.now();
	gps_comm.time.hour = now_time.hour();
	gps_comm.time.minute = now_time.minute();
	gps_comm.time.second = now_time.second();
	gps_comm.hdop = gps.HDOP;
	gps_comm.latitude = gps.latitude;
	gps_comm.longitude = gps.longitude;
	gps_comm.heading = 30;
	gps_comm.pitch = 60;
	gps_comm.roll = 90;
	SendToUpper((uint8_t *)&gps_comm, sizeof(gps_comm), 0x01);
	Serial.printf("[GPS]send lat:%f,yaw:%f,pitch:%f\n\r",gps_comm.latitude,gps_comm.heading,gps_comm.pitch);
	}else{
	Serial.println("[GPS]DataInvaild");
	}
#else
#ifndef ONLY_IMU
	gps.update();
	if (gps.latitude != 0 && gps.longitude != 0)
	{
		pack_gps_comm_frame();
		SendToUpper((uint8_t *)&gps_comm, sizeof(gps_comm), 0x01);
		Serial.printf("[GPS]send: lat:%f,yaw:%f,pitch:%f\n\r",gps_comm.latitude,gps_comm.heading,gps_comm.pitch);
	}else{
		Serial.println("[GPS]Data Invaild");
	}
#endif
#endif
}

int8_t BatteryFSM(){
	int sample = analogRead(BAT_ADC_IO);
	int8_t bat_per = (sample - 1670) * 100 / 550 ;
#ifdef DEBUG_MODE
	  Serial.printf("[Bat]raw:%d;per:%d\n",sample,bat_per);
#endif
	if(bat_per < 0){
		bat_per = 0;
		Serial.printf("[AutoSleep]LowBattery:%d\n",sample);
		DeepSleep();
	}
	if(bat_per > 100)bat_per = 100;
	return bat_per;
}
