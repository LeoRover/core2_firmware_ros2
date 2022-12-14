#pragma once

#include <rclc_parameter/rclc_parameter.h>

#include "diff_drive_lib/diff_drive_controller.hpp"

struct Parameters : diff_drive_lib::DiffDriveParams {
  // Override inherited parameters
  Parameters() {
    // Wheel
    wheel_encoder_resolution = 878.4F;
    wheel_torque_constant = 1.0F;
    wheel_pid_p = 0.0F;
    wheel_pid_i = 0.005F;
    wheel_pid_d = 0.0F;
    wheel_pwm_duty_limit = 100.0F;

    // Differential drive
    dd_wheel_radius = 0.0625F;
    dd_wheel_separation = 0.33F;
    dd_angular_velocity_multiplier = 1.91F;
    dd_input_timeout = 500;
  }

  float battery_min_voltage = 10.0;

  bool init(rclc_parameter_server_t* param_server);
  void update(rclc_parameter_server_t* param_server);
};
