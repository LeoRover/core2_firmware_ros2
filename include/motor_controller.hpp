#pragma once

#include <stdint.h>

#include "DCMotor.h"
#include "Encoder.h"

#include "diff_drive_lib/motor_controller_interface.hpp"

struct MotorConfiguration {
  int driver_num;
  MotNum motor_num;
  bool reverse_polarity;
};

class MotorController : public diff_drive_lib::MotorControllerInterface {
 public:
  MotorController(const MotorConfiguration& config) : config_(config){};

  void init() override;
  void setPWMDutyCycle(float pwm_duty) override;
  float getPWMDutyCycle() override;
  int32_t getEncoderCnt() override;
  void resetEncoderCnt() override;
  float getWindingCurrent() override;

 private:
  MotorConfiguration config_;

  DCMotor* motor_;
  Encoder* encoder_;

  float pwm_duty_ = 0.0F;
};
