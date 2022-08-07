#include "DRV8848_STM.h"

#include "motor_controller.hpp"

static const DRV8848_Params_t DEFAULT_MDRV1_PARAMS{
    MOT1A_IN, MOT1B_IN, MOT1_PWM,    MOT2A_IN,
    MOT2B_IN, MOT2_PWM, MOT12_FAULT, MOT12_SLEEP};

static const DRV8848_Params_t DEFAULT_MDRV2_PARAMS{
    MOT3A_IN, MOT3B_IN, MOT3_PWM,    MOT4A_IN,
    MOT4B_IN, MOT4_PWM, MOT34_FAULT, MOT34_SLEEP};

static DRV8848 mot_driver[] = {DRV8848(&DEFAULT_MDRV1_PARAMS),
                               DRV8848(&DEFAULT_MDRV2_PARAMS)};
static Encoder encoder[] = {Encoder(ENCODER_1), Encoder(ENCODER_2),
                            Encoder(ENCODER_3), Encoder(ENCODER_4)};

inline float clamp(const float value, const float limit) {
  if (value > limit)
    return limit;
  else if (value < -limit)
    return -limit;
  else
    return value;
}

void MotorController::init() {
  motor_ = mot_driver[config_.driver_num].getDCMotor(config_.motor_num);
  motor_->init(0);
  motor_->setPolarity(config_.reverse_polarity);
  motor_->setDriveMode(true);
  mot_driver[config_.driver_num].enable();
}

void MotorController::setPWMDutyCycle(float pwm_duty) {
  pwm_duty_ = clamp(pwm_duty, 100.0F);
  float power = pwm_duty_ / 100.0F;
  motor_->setPower(power);
}

float MotorController::getPWMDutyCycle() {
  return pwm_duty_;
}

int32_t MotorController::getEncoderCnt() {
  return encoder_->getCount();
}

void MotorController::resetEncoderCnt() {
  encoder_->resetCount();
}

float MotorController::getWindingCurrent() {
  return pwm_duty_;
}
