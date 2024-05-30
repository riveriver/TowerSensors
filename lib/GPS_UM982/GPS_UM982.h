#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
class GPS_UM982 : public Adafruit_GPS
{
public:
  GPS_UM982() : Adafruit_GPS(&Serial_Gps) {}
  const int baud = 115200;
  void init(void)
  {
    Serial_Gps.begin(baud, SWSERIAL_8N1, 5, 4, false,256);
    delay(500);
    Serial_Gps.println("FRESET");
    delay(2000);
    Serial_Gps.println("GPGGA COM1 0.2");
    delay(500);
    Serial_Gps.println("GPTRA COM1 0.2");
    delay(500);
    Serial_Gps.println("config com1 115200");
    delay(500);
    Serial_Gps.println("saveconfig");
    delay(500);
  };
  void update(void)
  {
    while(Serial_Gps.available()){
      char c = read();
      if (newNMEAreceived())
      {
#ifdef DEBUG_MODE
    Serial.println(lastNMEA());
#endif
        if (!parse(lastNMEA())){
          return;
        }
      }
    }
  };
  void test_SerialReceive(void)
  {
    String rx_str = "";
    while (Serial_Gps.available())
    {
      char ch = Serial_Gps.read();
      rx_str += ch;
    }
    if (rx_str.length() > 0)
    {
      Serial.println(rx_str);
    }
  }
  void test_Parse(void)
  {
    String GPSInfo = "$GNGGA,103250.00,2225.64091323,N,11412.50890811,E,1,17,1.0,24.5187,M,-2.1601,M,,*50";
    char charArray[GPSInfo.length() + 1];
    GPSInfo.toCharArray(charArray, sizeof(charArray));
    for (int i = 0; i < GPSInfo.length(); i++)
    {
      parse(charArray);
    }
  };

  String OutputGpsInfo(void)
  {
    /* time */
    String str = "[GPS]:";
      if (hour < 10)
      {
        str += '0';
      }
      str += String(hour, DEC);
      str += '-';
      if (minute < 10)
      {
        str += '0';
      }
      str += String(minute, DEC);
      str += '-';
      if (seconds < 10)
      {
        str += '0';
      }
      str += String(seconds, DEC);
      str += ",";
      // TRA
      str += String(headling);
      str += ",";
      str += String(pitch);
      str += ",";
      str += String(roll);
      str += ",";
      str += String(sol);
      str += ",";
      str += String(sat_num);
      // GGA
      str += ",";
      str += String(latitude, 4);
      str += ",";
      str += String(lat);
      str += ",";
      str += String(longitude, 4);
      str += ",";
      str += String(lon);
      str += ",";
      str += String(HDOP);
      str += ",";
      str += String((int)satellites);
    return str;
}

private:
  bool _debug = false;
  bool _echo = false;
  SoftwareSerial Serial_Gps;
  String test_info_str = "$GNGGA,103250.00,2225.64091323,N,11412.50890811,E,1,17,1.0,24.5187,M,-2.1601,M,,*50";
};