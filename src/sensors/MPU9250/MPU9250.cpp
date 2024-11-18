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

#include <string>

#include "MPU9250.h"
#include "RegisterMap.h"

#include "hal_compat.hpp"

void MPU9250::begin(uint32_t datarate) {
  _i2c.frequency(datarate);
}

uint8_t MPU9250::getMPU9250ID() {
  uint8_t c =
      readByte(MPU9250_ADDRESS,
               WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  return c;
}

void MPU9250::initMPU9250(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate) {
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1,
            0x00);  // Clear sleep mode bit (6), enable all sensors
  delay(100);       // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1,
            0x01);  // Auto select clock source to be PLL gyroscope reference if
                    // ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively; minimum delay time for this setting is 5.9 ms, which means
  // sensor fusion update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
  // kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV,
            sampleRate);  // Use a 200 Hz rate; a rate consistent with the
                          // filter update rate determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS,
                       GYRO_CONFIG);  // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02;        // Clear Fchoice bits [1:0]
  c = c & ~0x18;        // Clear AFS bits [4:3]
  c = c | Gscale << 3;  // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits
  // 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,
            c);  // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS,
               ACCEL_CONFIG);  // get current ACCEL_CONFIG register value
                               // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;               // Clear AFS bits [4:3]
  c = c | Ascale << 3;         // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,
            c);  // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is 1.13
  // kHz
  c = readByte(MPU9250_ADDRESS,
               ACCEL_CONFIG2);  // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F;  // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;   // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2,
            c);  // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS, and enable
  // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
  // controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG,
            0x10);  // INT is 50 microsecond pulse and any read to clear
  writeByte(MPU9250_ADDRESS, INT_ENABLE,
            0x01);  // Enable data ready (bit 0) interrupt
  delay(100);

  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x20);  // Enable I2C Master mode
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL,
            0x1D);  // I2C configuration STOP after each transaction, master I2C
                    // bus at 400 KHz
  writeByte(MPU9250_ADDRESS, I2C_MST_DELAY_CTRL,
            0x81);  // Use blocking data retreival and enable delay for mag
                    // sample rate mismatch
  writeByte(MPU9250_ADDRESS, I2C_SLV4_CTRL,
            0x01);  // Delay mag data retrieval to once every other accel/gyro
                    // data sample
}

float MPU9250::getGres(uint8_t Gscale) {
  switch (Gscale) {
      // Possible gyro scales (and their register bit settings) are:
      // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    case GFS_250DPS:
      _gRes = 250.0 / 32768.0;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0 / 32768.0;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0 / 32768.0;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0 / 32768.0;
      return _gRes;
      break;
    default:
      return 0;
  }
}

float MPU9250::getAres(uint8_t Ascale) {
  switch (Ascale) {
      // Possible accelerometer scales (and their register bit settings) are:
      // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
      // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
      // 2-bit value:
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
    default:
      return 0;
  }
}

void MPU9250::readMPU9250Data(int16_t* destination) {
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14,
            &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] =
      ((int16_t)rawData[0] << 8) |
      rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];
}

// I2C read/write functions for the MPU9250 sensors
void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  // Wire.beginTransmission(address);  // Initialize the Tx buffer
  // Wire.write(subAddress);           // Put slave register address in Tx
  // buffer Wire.write(data);                 // Put data in Tx buffer
  // Wire.endTransmission();           // Send the Tx buffer

  mpu9250_write_buffer[0] = subAddress;
  mpu9250_write_buffer[1] = data;
  _i2c.write((int)(address << 1), mpu9250_write_buffer, 2, 0);
}

uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress) {
  // uint8_t data = 0;                        // `data` will store the register
  // data Wire.beginTransmission(address);         // Initialize the Tx buffer
  // Wire.write(subAddress);                  // Put slave register address in
  // Tx buffer Wire.endTransmission(false);             // Send the Tx buffer,
  // but send a restart to keep connection alive Wire.requestFrom(address, 1);
  // // Read two bytes from slave register address on MPU9250 data =
  // Wire.read();                      // Fill Rx buffer with result return
  // data;                             // Return data read from slave register
  uint8_t data = 0;
  readBytes(address, subAddress, 1, &data);
  return data;
}

void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
                        uint8_t* dest) {
  // Wire.beginTransmission(address);   // Initialize the Tx buffer
  // Wire.write(subAddress);            // Put slave register address in Tx
  // buffer Wire.endTransmission(false);       // Send the Tx buffer, but send a
  // restart to keep connection alive uint8_t i = 0; Wire.requestFrom(address,
  // count);  // Read bytes from slave register address while (Wire.available())
  // {
  //       dest[i++] = Wire.read(); }         // Put read results in the Rx
  //       buffer
  mpu9250_write_buffer[0] = subAddress;
  _i2c.write((int)(address << 1), mpu9250_write_buffer, 1, 1);
  _i2c.read((int)(address << 1), (char*)dest, count, 0);
}
