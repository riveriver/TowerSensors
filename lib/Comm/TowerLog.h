// #include "Arduino.h"
// #include <SoftwareSerial.h>
// class TowerLog {
// public:
//   const int baud = 115200;
//   const uint8_t rx_pin = 18;
//   const uint8_t tx_pin = 17;
//   void init(void)
//   {
//     Serial_Log.begin(baud, SWSERIAL_8N1, rx_pin, tx_pin, false);
//     delay(500);
//   }
//   void Log(String str){
//     Serial_Log.println(str);
//   }
// private:
//   bool _debug_mode = true;
//   SoftwareSerial Serial_Log;
// };