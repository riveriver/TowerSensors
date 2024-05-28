#include <Arduino.h>
#include <SoftwareSerial.h>
#include <REG.h>
#include <wit_c_sdk.h>
// SoftwareSerial Serial_IMU;
HardwareSerial Serial_IMU(1);

    #define RE_IO 37
    #define ACC_UPDATE		0x01
    #define GYRO_UPDATE		0x02
    #define ANGLE_UPDATE	0x04
    #define MAG_UPDATE		0x08
    #define READ_UPDATE		0x80
    char s_cDataUpdate = 0;
    float fAcc[3], fGyro[3], fAngle[3];
    float quat_raw[4] = {0};
    void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
    {
        // digitalWrite(RE_IO, HIGH);
        Serial_IMU.write(p_data, uiSize);
        Serial_IMU.flush();
        // digitalWrite(RE_IO, LOW);
    }

    /**
     * @brief 处理传感器数据更新。
     * 
     * 该函数根据传入的寄存器地址和数量，更新相应的传感器数据标志位。
     * 根据寄存器的不同值，置位不同的数据更新标志。
     * 
     * @param uiReg 寄存器地址。该参数指定了首先被检查的寄存器地址。
     * @param uiRegNum 要处理的寄存器数量。函数将对从uiReg开始的连续uiRegNum个寄存器地址进行处理。
     */
    static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
    {
        int i;
        // 遍历指定数量的寄存器
        for(i = 0; i < uiRegNum; i++)
        {
            switch(uiReg)
            {
                case AZ:
                    // 更新加速度数据标志位
                    s_cDataUpdate |= ACC_UPDATE;
                break;
                case GZ:
                    // 更新陀螺仪数据标志位
                    s_cDataUpdate |= GYRO_UPDATE;
                break;
                case HZ:
                    // 更新磁力计数据标志位
                    s_cDataUpdate |= MAG_UPDATE;
                break;
                case Yaw:
                    // 更新角度数据标志位
                    s_cDataUpdate |= ANGLE_UPDATE;
                break;
                case addr_q0:
       
                break;
                default:
                    // 更新一般读取数据标志位
                    s_cDataUpdate |= READ_UPDATE;
                break;
            }
            // 处理下一个寄存器
            uiReg++;
        }
    }

    static void Delayms(uint16_t ucMs)
    {
    delay(ucMs);
    }

    void test_receive_parse(uint8_t addr){
        changeAddr(addr);
        WitReadReg(addr_q0, 4);
        // 10ms
        delay(10);
        String str = "imu_rx:";
        while (Serial_IMU.available())
        {
            byte rx_byte = Serial_IMU.read();
            WitSerialDataIn(rx_byte);
            str += " 0x" + String(rx_byte,HEX);
        }
#ifdef DEBUG_MODE
        Serial.println(str);
#endif
        for(int i = 0; i < 4; i++)
        {
            quat_raw[i] = sReg[addr_q0+i] / 32768.0f;
        }       
    }

    void RS485_IO_Init(void)
    {
    // pinMode(RE_IO, OUTPUT);
    }
    void test_raw_send_receive(){
        uint8_t ucBuff[8] = {0x55,0x03,0x00,0x30,0x00,0x30,0x48,0x05};
        WitReadCustom(ucBuff);
        delay(2000);
        Serial.printf("imu_raw:");
        while (Serial_IMU.available())
        {
            Serial.printf("0x%x ",Serial_IMU.read());
        }
        Serial.println();
    }
    void Imu_initHwt9073(uint8_t addr){
        Serial_IMU.begin(9600,SERIAL_8N1,38,37,false);
        delay(100);
        WitInit(WIT_PROTOCOL_MODBUS, addr);
        WitSerialWriteRegister(SensorUartSend);
        WitRegisterCallBack(CopeSensorData);
        WitDelayMsRegister(Delayms);
    };
