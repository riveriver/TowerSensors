/*I2C 模式最高支持 400Kbps，8bit 寄存器地址，8bit 寄存器数据，发送和接收都是高位在前
（MSB），I2C 设备地址：写为 0xEA，读为 0xEB*/

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h> 
class IP2315{
private:
  enum class REG {
    IIC_ADDRESS = 0x75,
    READ_ADDRESS = 0xEB,
    WRITE_ADDRESS = 0xEA,
    SYS_CTL1 = 0x01,
    CHG_STAT = 0xC7,
  };
  TwoWire* wire;
public:

    Adafruit_I2CDevice *i2c_dev = NULL;
    bool begin(TwoWire *wireInstance) {
    if (i2c_dev)
        delete i2c_dev;
    i2c_dev = new Adafruit_I2CDevice(0x75, wireInstance);
    if (!i2c_dev->begin())
        return false;
    return true;
    }
    uint8_t read_register(uint8_t reg) {
        // read a register by writing first, then reading
        // byte buffer[3] = {0xEA,reg,0xEB};  // we'll reuse the same buffer
        // i2c_dev->write_then_read(buffer, 3, buffer, 1, false);
        // return buffer[0];
        
        byte buffer[1] = {0xEA};
        if (!i2c_dev->write(buffer, 1, true)) {
         return false;
        }

        buffer[0] = {reg};
        if (!i2c_dev->write(buffer, 1, true)) {
         return false;
        }

        buffer[0] = {0xEB};
        if (!i2c_dev->write(buffer, 1, true)) {
         return false;
        }

        return i2c_dev->read(buffer,1,true);

    }
    // uint8_t read_register2(uint8_t reg) {
    //     uint8_t value;
    //     i2c_dev->beginTransmission(0x75); // 替换 DEVICE_ADDRESS 为从设备的地址
    //     i2c_dev->write(0xEA); // 替换 dataToSend 为要发送的数据
    //     i2c_dev->endTransmission();
    
    //     // 等待从设备的应答
    //     while (i2c_dev->available() == 0) {
    //         // 等待从设备准备好
    //     }

    //     i2c_dev->beginTransmission(0x75); // 替换 DEVICE_ADDRESS 为从设备的地址
    //     i2c_dev->write(reg); // 替换 dataToSend 为要发送的数据
    //     i2c_dev->endTransmission();

    //    // 等待从设备的应答
    //     while (i2c_dev->available() == 0) {
    //         // 等待从设备准备好
    //     }

    //     i2c_dev->beginTransmission(0x75); // 替换 DEVICE_ADDRESS 为从设备的地址
    //     i2c_dev->write(0xEB); // 替换 dataToSend 为要发送的数据
    //     i2c_dev->endTransmission();

    //    // 等待从设备的应答
    //     while (i2c_dev->available() == 0) {
    //         // 等待从设备准备好
    //     }

    //     // 读取从设备的应答数据
        
    //     value = i2c_dev->read();
    //     Wire.endTransmission(false); // 发送 NACK 信号，而不是发送 STOP 信号
    //     return value;
    // }
    // uint8_t read_register3(uint8_t reg) {
    //     uint8_t buffer[2] = {0xEB,reg};
    //     i2c_dev->write(buffer,2,true,);
    //     delay(2);
    //     return buffer[0];
    // }

//   void write(const REG reg, const bool stop = true) {
//     wire->beginTransmission(0xEA);
//     wire->write((uint8_t)reg);
//     wire->endTransmission(stop);
//   }

//     byte read(const REG reg) {
//         byte data = 0;
//         write(reg, false);
//         wire->requestFrom((uint8_t)I2C_ADDR, (uint8_t)1);
//         if (wire->available() == 1) data = wire->read();
//         return data;
//     }
};
