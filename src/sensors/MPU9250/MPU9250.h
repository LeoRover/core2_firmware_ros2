/* 06/16/2017 Copyright Tlera Corporation
 *
 *  Created by Kris Winer
 *
 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Addition of 9 DoF sensor fusion using
 open source Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V
 Dragonfly STM32L476 Breakout Board.

 Library may be used freely and without limit with attribution.

*/

#ifndef MPU9250_h
#define MPU9250_h

#include <cstdint>

#include <mbed.h>

class MPU9250 {
 public:
  MPU9250(mbed::I2C& i2c) : _i2c(i2c) {
    // _i2c.start();
  }

  void begin(uint32_t datarate);
  uint8_t getMPU9250ID();
  void initMPU9250(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate);
  float getAres(uint8_t Ascale);
  float getGres(uint8_t Gscale);
  void readMPU9250Data(int16_t* destination);
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
                 uint8_t* dest);

 private:
  mbed::I2C& _i2c;
  float _aRes;
  float _gRes;
  char mpu9250_write_buffer[128];
};

#endif
