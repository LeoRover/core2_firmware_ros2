#pragma once

#include <mbed.h>

#include <sensors/MPU9250/MPU9250.h>

class ImuReceiver {
 public:
  explicit ImuReceiver(mbed::I2C &i2c) : mpu_(i2c) {
    mpu_.begin(100000);
  }

  void start() {
    thread_running_ = true;
    thread_.start(mbed::callback(this, &ImuReceiver::loop));
  }

  void stop() {
    thread_running_ = false;
    thread_.join();
  }

  bool is_initialized() {
    return initialized_;
  }

  float temp;        // temperature
  float ax, ay, az;  // accelerometer data
  float gx, gy, gz;  // gyroscope data

 private:
  MPU9250 mpu_;
  float ares_, gres_;
  rtos::Thread thread_;
  bool initialized_, thread_running_;
  void loop();
  bool init();
  void update();
  bool check_status();
};