#include "imu_receiver.hpp"
#include <sensors/MPU9250/RegisterMap.h>

static constexpr float PI = 3.141592653F;
static constexpr float GRAVITATIONAL_ACCELERATION = 9.80665F;
static constexpr float DEGREE_TO_RADIAN = 2.0F * PI / 360.0F;
static constexpr float TEMP_RESOLUTION = 1.0F / 333.87F;
static constexpr float TEMP_OFFSET = 21.0F;
static constexpr uint16_t CHECK_PERIOD = 500;
static uint16_t cnt = 0;

bool ImuReceiver::init() {
  if (!check_status()) return false;

  mpu_.initMPU9250(AFS_2G, GFS_250DPS, 0x02);

  ares_ = mpu_.getAres(AFS_2G);
  gres_ = mpu_.getGres(GFS_250DPS);

  temp = 0.0;
  ax = ay = az = 0.0;
  gx = gy = gz = 0.0;

  cnt = 0;

  return true;
}

void ImuReceiver::update() {
  int16_t MPU9250Data[7];
  mpu_.readMPU9250Data(MPU9250Data);

  temp = (static_cast<float>(MPU9250Data[3]) - TEMP_OFFSET) * TEMP_RESOLUTION +
         TEMP_OFFSET;
  ax = static_cast<float>(MPU9250Data[0]) * ares_ * GRAVITATIONAL_ACCELERATION;
  ay = static_cast<float>(MPU9250Data[1]) * ares_ * GRAVITATIONAL_ACCELERATION;
  az = static_cast<float>(MPU9250Data[2]) * ares_ * GRAVITATIONAL_ACCELERATION;
  gx = static_cast<float>(MPU9250Data[4]) * gres_ * DEGREE_TO_RADIAN;
  gy = static_cast<float>(MPU9250Data[5]) * gres_ * DEGREE_TO_RADIAN;
  gz = static_cast<float>(MPU9250Data[6]) * gres_ * DEGREE_TO_RADIAN;
}

bool ImuReceiver::check_status() {
  uint8_t c = mpu_.getMPU9250ID();

  return c == 0x71;
}

void ImuReceiver::loop() {
  while (thread_running_) {
    if (is_initialized()) {
      update();
    } else {
      initialized_ = init();
    }

    cnt++;
    if(!cnt % CHECK_PERIOD) {
      initialized_ = check_status();
    }

    ThisThread::sleep_for(10);
  }
}