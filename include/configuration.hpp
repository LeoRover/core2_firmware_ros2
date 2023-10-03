#pragma once

#include "diff_drive_lib/robot_controller.hpp"

#include "motor_controller.hpp"

// Size of the heap memory used for micro-ROS entities
constexpr uint32_t UROS_HEAP_SIZE = 30000;

// Domain ID used for ROS communication
constexpr size_t ROS_DOMAIN_ID = 0;

// Name of the ROS node
constexpr const char* ROS_NODE_NAME = "firmware";

// Namespace of the ROS node
constexpr const char* ROS_NAMESPACE = "";

// Number of encoder readings to remember when estimating the wheel velocity
constexpr uint32_t ENCODER_BUFFER_SIZE = 10;

// The period (in number of calls to the update() function) at which the battery
// voltage is probed
constexpr uint8_t BATTERY_PROBE_PERIOD = 10;

// Number of battery voltage readings to average
constexpr uint32_t BATTERY_BUFFER_SIZE = 300;

// Informative LED GPIO
constexpr PinName LED_PIN = EXT_PIN1;

// The period (in milliseconds) between calls to the update() function
constexpr uint16_t UPDATE_PERIOD = 10;

// The periods (in number of calls to the update() function) at which different
// data is publihed on the ROS topics
constexpr uint8_t BATTERY_PUB_PERIOD = 10;
constexpr uint8_t JOINTS_PUB_PERIOD = 5;
constexpr uint8_t ODOM_PUB_PERIOD = 5;
// constexpr uint8_t IMU_PUB_PERIOD = 1;

// Motor driver configurations
constexpr MotorConfiguration MOT_A_CONFIG = {
    .driver_num = 0,
    .motor_num = MOT1,
    .reverse_polarity = false,
};

constexpr MotorConfiguration MOT_B_CONFIG = {
    .driver_num = 0,
    .motor_num = MOT2,
    .reverse_polarity = false,
};

constexpr MotorConfiguration MOT_C_CONFIG = {
    .driver_num = 1,
    .motor_num = MOT1,
    .reverse_polarity = true,
};

constexpr MotorConfiguration MOT_D_CONFIG = {
    .driver_num = 1,
    .motor_num = MOT2,
    .reverse_polarity = true,
};

extern MotorController MotA;
extern MotorController MotB;
extern MotorController MotC;
extern MotorController MotD;

constexpr diff_drive_lib::RobotConfiguration ROBOT_CONFIG = {
    .wheel_FL_conf =
        {
            .motor = MotC,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
            .velocity_rolling_window_size = ENCODER_BUFFER_SIZE,
        },
    .wheel_RL_conf =
        {
            .motor = MotD,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
            .velocity_rolling_window_size = ENCODER_BUFFER_SIZE,
        },
    .wheel_FR_conf =
        {
            .motor = MotA,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
            .velocity_rolling_window_size = ENCODER_BUFFER_SIZE,
        },
    .wheel_RR_conf =
        {
            .motor = MotB,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
            .velocity_rolling_window_size = ENCODER_BUFFER_SIZE,
        },
};