#include <atomic>

#include <mbed.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <geometry_msgs/msg/twist.h>
#include <leo_msgs/msg/imu.h>
#include <leo_msgs/msg/wheel_odom.h>
#include <leo_msgs/msg/wheel_odom_mecanum.h>
#include <leo_msgs/msg/wheel_states.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/trigger.h>

#include "diff_drive_lib/diff_drive_controller.hpp"
#include "diff_drive_lib/mecanum_controller.hpp"
#include "diff_drive_lib/wheel_controller.hpp"

#include "microros/serial_transport.hpp"

#include "configuration.hpp"
#include "hal_compat.hpp"
#include "microros_allocators.hpp"
#include "parameters.hpp"

static rcl_allocator_t microros_allocator =
    rcutils_get_zero_initialized_allocator();
static rcl_init_options_t init_options;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rclc_parameter_server_t param_server;
static rcl_timer_t ping_timer, sync_timer;

static std_msgs__msg__Float32 battery;
static std_msgs__msg__Float32 battery_averaged;
static rcl_publisher_t battery_pub;
static rcl_publisher_t battery_averaged_pub;
static analogin_t battery_adc;
static float battery_buffer_memory[BATTERY_BUFFER_SIZE];
static diff_drive_lib::CircularBuffer<float> battery_buffer(
    BATTERY_BUFFER_SIZE, battery_buffer_memory);
static std::atomic_bool publish_battery(false);

static leo_msgs__msg__WheelOdom wheel_odom;
static rcl_publisher_t wheel_odom_pub;
static leo_msgs__msg__WheelOdomMecanum wheel_odom_mecanum;
static rcl_publisher_t wheel_odom_mecanum_pub;
static std::atomic_bool publish_wheel_odom(false);

static leo_msgs__msg__WheelStates wheel_states;
static rcl_publisher_t wheel_states_pub;
static std::atomic_bool publish_wheel_states(false);

// static leo_msgs__msg__Imu imu;
// static rcl_publisher_t imu_pub;
// static std::atomic_bool publish_imu(false);

static rcl_subscription_t twist_sub;
static geometry_msgs__msg__Twist twist_msg;

static std_msgs__msg__Empty param_trigger;
static rcl_publisher_t param_trigger_pub;
static std::atomic_bool publish_param_trigger(true);

#define WHEEL_WRAPPER(NAME)                         \
  constexpr const char* NAME##_cmd_pwm_topic =      \
      "~/wheel_" #NAME "/cmd_pwm_duty";             \
  constexpr const char* NAME##_cmd_vel_topic =      \
      "~/wheel_" #NAME "/cmd_velocity";             \
  static rcl_subscription_t NAME##_cmd_pwm_sub;     \
  static rcl_subscription_t NAME##_cmd_vel_sub;     \
  static std_msgs__msg__Float32 NAME##_cmd_pwm_msg; \
  static std_msgs__msg__Float32 NAME##_cmd_vel_msg;

WHEEL_WRAPPER(FL)
WHEEL_WRAPPER(RL)
WHEEL_WRAPPER(FR)
WHEEL_WRAPPER(RR)

static rcl_service_t reset_odometry_srv, firmware_version_srv, board_type_srv,
    reset_board_srv, boot_firmware_srv;
static std_srvs__srv__Trigger_Request reset_odometry_req, firmware_version_req,
    board_type_req, reset_board_req, boot_firmware_req;
static std_srvs__srv__Trigger_Response reset_odometry_res, firmware_version_res,
    board_type_res, reset_board_res, boot_firmware_res;

static std::atomic_bool reset_request(false);
static std::atomic_bool boot_request(false);
static DigitalOut LED(LED_PIN);

enum class AgentStatus {
  BOOT,
  CONNECTING_TO_AGENT,
  AGENT_CONNECTED,
  AGENT_LOST
};
static AgentStatus status = AgentStatus::CONNECTING_TO_AGENT;

enum class BatteryLedStatus {
  LOW_BATTERY,
  NOT_CONNECTED,
  CONNECTED,
  BOOT,
};
static BatteryLedStatus battery_led_status = BatteryLedStatus::NOT_CONNECTED;

MotorController MotA(MOT_A_CONFIG);
MotorController MotB(MOT_B_CONFIG);
MotorController MotC(MOT_C_CONFIG);
MotorController MotD(MOT_D_CONFIG);

static diff_drive_lib::RobotController* controller;
// static ImuReceiver imu_receiver(&IMU_I2C);

static Parameters params;
static std::atomic_bool reload_parameters(false);

static void cmdVelCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg =
      (const geometry_msgs__msg__Twist*)msgin;
  controller->setSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
}

static void resetOdometryCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  controller->resetOdom();
  res->success = true;
}

static void resetBoardCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  reset_request = true;
  rosidl_runtime_c__String__assign(&res->message,
                                   "Requested board software reset");
  res->success = true;
}

static void getFirmwareVersionCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  rosidl_runtime_c__String__assign(&res->message, FIRMWARE_VERSION);
  res->success = true;
}

static void getBoardTypeCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  rosidl_runtime_c__String__assign(&res->message, "core2");
  res->success = true;
}

static void wheelCmdPWMDutyCallback(const void* msgin, void* context) {
  const std_msgs__msg__Float32* msg = (std_msgs__msg__Float32*)msgin;
  diff_drive_lib::WheelController* wheel =
      (diff_drive_lib::WheelController*)context;
  wheel->disable();
  wheel->motor.setPWMDutyCycle(msg->data);
}

static void wheelCmdVelCallback(const void* msgin, void* context) {
  const std_msgs__msg__Float32* msg = (std_msgs__msg__Float32*)msgin;
  diff_drive_lib::WheelController* wheel =
      (diff_drive_lib::WheelController*)context;
  wheel->enable();
  wheel->setTargetVelocity(msg->data);
}

static void bootFirmwareCallback(const void* reqin, void* resin) {
  std_srvs__srv__Trigger_Response* res =
      (std_srvs__srv__Trigger_Response*)resin;
  boot_request = true;
  rosidl_runtime_c__String__assign(&res->message, "Requested firmware boot.");
  res->success = true;
}

static bool parameterChangedCallback(const Parameter*, const Parameter*,
                                     void*) {
  reload_parameters = true;
  return true;
}

static UARTSerial uros_serial(RPI_SERIAL_TX, RPI_SERIAL_RX);

static void pingTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK)
    status = AgentStatus::AGENT_LOST;
}

static void syncTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  rmw_uros_sync_session(1000);
}

static void initMsgs() {
  std_msgs__msg__Float32__init(&battery);
  std_msgs__msg__Float32__init(&battery_averaged);
  leo_msgs__msg__WheelOdom__init(&wheel_odom);
  leo_msgs__msg__WheelOdomMecanum__init(&wheel_odom_mecanum);
  leo_msgs__msg__WheelStates__init(&wheel_states);
  // leo_msgs__msg__Imu__init(&imu);
  std_msgs__msg__Empty__init(&param_trigger);
}

#define RCCHECK(fn) \
  if ((fn != RCL_RET_OK)) return false;

static bool initROS() {
  // Init options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, microros_allocator))
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID))

  // Support
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &microros_allocator))

  // Node
  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NAMESPACE, &support))

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context,
                             16 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
                             &microros_allocator))

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "~/battery"))
  RCCHECK(rclc_publisher_init_best_effort(
      &battery_averaged_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "~/battery_averaged"))
  RCCHECK(rclc_publisher_init_best_effort(
      &wheel_states_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelStates),
      "~/wheel_states"))
  // RCCHECK(rclc_publisher_init_best_effort(
  //     &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, Imu),
  //     "~/imu"))
  RCCHECK(rclc_publisher_init_best_effort(
      &param_trigger_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "~/param_trigger"))

  // Subscriptions
  RCCHECK(rclc_subscription_init_default(
      &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"))
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg,
                                         cmdVelCallback, ON_NEW_DATA))

#define WHEEL_INIT_ROS(NAME)                                            \
  RCCHECK(rclc_subscription_init_default(                               \
      &NAME##_cmd_pwm_sub, &node,                                       \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),              \
      NAME##_cmd_pwm_topic))                                            \
  RCCHECK(rclc_executor_add_subscription_with_context(                  \
      &executor, &NAME##_cmd_pwm_sub, &NAME##_cmd_pwm_msg,              \
      wheelCmdPWMDutyCallback, &controller->wheel_##NAME, ON_NEW_DATA)) \
  RCCHECK(rclc_subscription_init_default(                               \
      &NAME##_cmd_vel_sub, &node,                                       \
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),              \
      NAME##_cmd_vel_topic))                                            \
  RCCHECK(rclc_executor_add_subscription_with_context(                  \
      &executor, &NAME##_cmd_vel_sub, &NAME##_cmd_vel_msg,              \
      wheelCmdVelCallback, &controller->wheel_##NAME, ON_NEW_DATA))

  WHEEL_INIT_ROS(FL)
  WHEEL_INIT_ROS(RL)
  WHEEL_INIT_ROS(FR)
  WHEEL_INIT_ROS(RR)

  // Services
  RCCHECK(rclc_service_init_default(
      &reset_odometry_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/reset_odometry"))
  RCCHECK(rclc_executor_add_service(&executor, &reset_odometry_srv,
                                    &reset_odometry_req, &reset_odometry_res,
                                    resetOdometryCallback))
  RCCHECK(rclc_service_init_default(
      &firmware_version_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
      "~/get_firmware_version"))
  RCCHECK(rclc_executor_add_service(
      &executor, &firmware_version_srv, &firmware_version_req,
      &firmware_version_res, getFirmwareVersionCallback))
  RCCHECK(rclc_service_init_default(
      &board_type_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/get_board_type"))
  RCCHECK(rclc_executor_add_service(&executor, &board_type_srv, &board_type_req,
                                    &board_type_res, getBoardTypeCallback))
  RCCHECK(rclc_service_init_default(
      &reset_board_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/reset_board"))
  RCCHECK(rclc_executor_add_service(&executor, &reset_board_srv,
                                    &reset_board_req, &reset_board_res,
                                    resetBoardCallback))
  RCCHECK(rclc_service_init_default(
      &boot_firmware_srv, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "~/boot"))
  RCCHECK(rclc_executor_add_service(&executor, &boot_firmware_srv,
                                    &boot_firmware_req, &boot_firmware_res,
                                    &bootFirmwareCallback))

  // Parameter Server
  static rclc_parameter_options_t param_options;
  param_options.max_params = 12;
  param_options.notify_changed_over_dds = true;
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node,
                                                 &param_options))
  if (!params.init(&param_server)) return false;
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server,
                                             parameterChangedCallback))

  // Timers
  RCCHECK(rclc_timer_init_default(&ping_timer, &support, RCL_MS_TO_NS(5000),
                                  pingTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &ping_timer))
  RCCHECK(rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(60000),
                                  syncTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer))

  // Allocate memory
  RCCHECK(rclc_executor_prepare(&executor))

  return true;
}

static void finiROS() {
  rclc_executor_fini(&executor);
  rclc_parameter_server_fini(&param_server, &node);
  (void)!rcl_timer_fini(&ping_timer);
  (void)!rcl_timer_fini(&sync_timer);
  (void)!rcl_service_fini(&reset_board_srv, &node);
  (void)!rcl_service_fini(&board_type_srv, &node);
  (void)!rcl_service_fini(&firmware_version_srv, &node);
  (void)!rcl_service_fini(&reset_odometry_srv, &node);
  (void)!rcl_service_fini(&boot_firmware_srv, &node);
  (void)!rcl_subscription_fini(&twist_sub, &node);
  (void)!rcl_subscription_fini(&FL_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&RL_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&FR_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&RR_cmd_vel_sub, &node);
  (void)!rcl_subscription_fini(&FL_cmd_pwm_sub, &node);
  (void)!rcl_subscription_fini(&RL_cmd_pwm_sub, &node);
  (void)!rcl_subscription_fini(&FR_cmd_pwm_sub, &node);
  (void)!rcl_subscription_fini(&RR_cmd_pwm_sub, &node);
  // (void)!rcl_publisher_fini(&imu_pub, &node);
  (void)!rcl_publisher_fini(&wheel_states_pub, &node);
  (void)!rcl_publisher_fini(&wheel_odom_pub, &node);
  (void)!rcl_publisher_fini(&wheel_odom_mecanum_pub, &node);
  (void)!rcl_publisher_fini(&battery_averaged_pub, &node);
  (void)!rcl_publisher_fini(&battery_pub, &node);
  (void)!rcl_publisher_fini(&param_trigger_pub, &node);
  (void)!rcl_node_fini(&node);
  (void)!rcl_init_options_fini(&init_options);
  rclc_support_fini(&support);

  free_all_heap();
}

extern uint32_t encoder_gpio_pull;

static void setup() {
  microros_allocator.allocate = microros_allocate;
  microros_allocator.deallocate = microros_deallocate;
  microros_allocator.reallocate = microros_reallocate;
  microros_allocator.zero_allocate = microros_zero_allocate;

  (void)!rcutils_set_default_allocator(&microros_allocator);

  set_microros_serial_transports(&uros_serial);

  initMsgs();

  analogin_init(&battery_adc, BAT_MEAS);

  encoder_gpio_pull = GPIO_PULLUP;

  status = AgentStatus::CONNECTING_TO_AGENT;
  battery_led_status = BatteryLedStatus::NOT_CONNECTED;
}

bool initController() {
  if (params.mecanum_wheels) {
    RCCHECK(rclc_publisher_init_best_effort(
        &wheel_odom_mecanum_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelOdomMecanum),
        "~/wheel_odom_mecanum"))
    controller = new diff_drive_lib::MecanumController(ROBOT_CONFIG);
  } else {
    RCCHECK(rclc_publisher_init_best_effort(
        &wheel_odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(leo_msgs, msg, WheelOdom), "~/wheel_odom"))
    controller = new diff_drive_lib::DiffDriveController(ROBOT_CONFIG);
  }
  controller->init(params);
  return true;
}

static void loop() {
  // static uint32_t boot_enter_time;
  static Timer boot_timer;
  switch (status) {
    case AgentStatus::CONNECTING_TO_AGENT:
      // Try to connect to uros agent
      if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
        if (initROS()) {
          (void)!rcl_timer_call(&sync_timer);
          boot_timer.start();
          status = AgentStatus::BOOT;
        } else
          finiROS();
      }
      break;
    case AgentStatus::BOOT:
      rclc_executor_spin_some(&executor, 0);

      if (reload_parameters.exchange(false)) {
        params.update(&param_server);
      }

      if (publish_param_trigger) {
        (void)!rcl_publish(&param_trigger_pub, &param_trigger, NULL);
        publish_param_trigger = false;
      } else if (boot_request || boot_timer.read_ms() >= BOOT_TIMEOUT) {
        (void)!rcl_publisher_fini(&param_trigger_pub, &node);
        // this uncomented breaks whole ROS communication
        // (void)!rclc_executor_remove_service(&executor, &boot_firmware_srv);
        // (void)!rcl_service_fini(&boot_firmware_srv, &node);
        (void)!initController();
        boot_timer.stop();
        boot_timer.reset();
        status = AgentStatus::AGENT_CONNECTED;
      }
      break;
    case AgentStatus::AGENT_CONNECTED:
      rclc_executor_spin_some(&executor, 0);

      if (reset_request) reset();

      if (publish_battery) {
        (void)!rcl_publish(&battery_pub, &battery, NULL);
        (void)!rcl_publish(&battery_averaged_pub, &battery_averaged, NULL);
        publish_battery = false;
      }

      if (publish_wheel_odom) {
        if (params.mecanum_wheels) {
          (void)!rcl_publish(&wheel_odom_mecanum_pub, &wheel_odom_mecanum,
                             NULL);
        } else {
          (void)!rcl_publish(&wheel_odom_pub, &wheel_odom, NULL);
        }
        publish_wheel_odom = false;
      }

      if (publish_wheel_states) {
        (void)!rcl_publish(&wheel_states_pub, &wheel_states, NULL);
        publish_wheel_states = false;
      }

      // if (publish_imu) {
      //   (void)!rcl_publish(&imu_pub, &imu, NULL);
      //   publish_imu = false;
      // }

      if (reload_parameters.exchange(false)) {
        params.update(&param_server);
        controller->updateParams(params);
      }
      break;
    case AgentStatus::AGENT_LOST:
      controller->disable();
      finiROS();
      status = AgentStatus::CONNECTING_TO_AGENT;
      break;
    default:
      break;
  }
}

static builtin_interfaces__msg__Time now() {
  const int64_t nanos = rmw_uros_epoch_nanos();
  builtin_interfaces__msg__Time stamp;
  stamp.sec = nanos / (1000 * 1000 * 1000);
  stamp.nanosec = nanos % (1000 * 1000 * 1000);
  return stamp;
}

void update_battery_led(uint32_t cnt) {
  static bool blinking = false;
  static uint8_t blinks_cnt = 0;

  switch (battery_led_status) {
    case BatteryLedStatus::LOW_BATTERY:
      if (cnt % 10 == 0) gpio_toggle(LED);
      break;
    case BatteryLedStatus::NOT_CONNECTED:
      if (cnt % 50 == 0) gpio_toggle(LED);
      break;
    case BatteryLedStatus::CONNECTED:
      gpio_reset(LED);
      break;
    case BatteryLedStatus::BOOT:
      if (blinking) {
        if (cnt % 10 == 0) {
          gpio_toggle(LED);
          ++blinks_cnt;
        }
        if (blinks_cnt >= 4) {
          blinking = false;
          blinks_cnt = 0;
        }
      } else {
        gpio_reset(LED);
        if (cnt % 100 == 0) blinking = true;
      }
      break;
    default:
      break;
  }
}

static void update() {
  static uint32_t cnt = 0;
  ++cnt;

  static float battery_sum = 0.0F;
  static float battery_avg = 0.0F;
  float battery_new = 3.3f * VIN_MEAS_CORRECTION *
                      (UPPER_RESISTOR + LOWER_RESISTOR) / LOWER_RESISTOR *
                      analogin_read(&battery_adc);

  if (cnt % BATTERY_PROBE_PERIOD == 0) {
    battery_sum += battery_new;
    battery_sum -= battery_buffer.push_back(battery_new);
    battery_avg =
        battery_sum / static_cast<float>(std::min(BATTERY_BUFFER_SIZE,
                                                  cnt / BATTERY_PROBE_PERIOD));
  }

  if (battery_avg < params.battery_min_voltage) {
    battery_led_status = BatteryLedStatus::LOW_BATTERY;
  } else {
    if (status == AgentStatus::BOOT) {
      battery_led_status = BatteryLedStatus::BOOT;
    } else if (status != AgentStatus::AGENT_CONNECTED) {
      battery_led_status = BatteryLedStatus::NOT_CONNECTED;
    } else {
      battery_led_status = BatteryLedStatus::CONNECTED;
    }
  }

  update_battery_led(cnt);

  if (status == AgentStatus::BOOT) {
    if (cnt % PARAM_TRIGGER_PUB_PERIOD == 0 && !publish_param_trigger) {
      publish_param_trigger = true;
    }
  }

  if (status != AgentStatus::AGENT_CONNECTED) return;

  controller->update(UPDATE_PERIOD);

  if (cnt % BATTERY_PUB_PERIOD == 0 && !publish_battery) {
    battery.data = battery_new;
    battery_averaged.data = battery_avg;

    publish_battery = true;
  }

  if (cnt % JOINTS_PUB_PERIOD == 0 && !publish_wheel_states) {
    auto robot_wheel_states = controller->getWheelStates();

    wheel_states.stamp = now();
    for (size_t i = 0; i < 4; i++) {
      wheel_states.position[i] = robot_wheel_states.position[i];
      wheel_states.velocity[i] = robot_wheel_states.velocity[i];
      wheel_states.torque[i] = robot_wheel_states.torque[i];
      wheel_states.pwm_duty_cycle[i] = robot_wheel_states.pwm_duty_cycle[i];
    }

    publish_wheel_states = true;
  }

  if (cnt % ODOM_PUB_PERIOD == 0 && !publish_wheel_odom) {
    auto robot_odom = controller->getOdom();

    if (params.mecanum_wheels) {
      wheel_odom_mecanum.stamp = now();
      wheel_odom_mecanum.velocity_lin_x = robot_odom.velocity_lin_x;
      wheel_odom_mecanum.velocity_lin_y = robot_odom.velocity_lin_y;
      wheel_odom_mecanum.velocity_ang = robot_odom.velocity_ang;
      wheel_odom_mecanum.pose_x = robot_odom.pose_x;
      wheel_odom_mecanum.pose_y = robot_odom.pose_y;
      wheel_odom_mecanum.pose_yaw = robot_odom.pose_yaw;
    } else {
      wheel_odom.stamp = now();
      wheel_odom.velocity_lin = robot_odom.velocity_lin_x;
      wheel_odom.velocity_ang = robot_odom.velocity_ang;
      wheel_odom.pose_x = robot_odom.pose_x;
      wheel_odom.pose_y = robot_odom.pose_y;
      wheel_odom.pose_yaw = robot_odom.pose_yaw;
    }
    publish_wheel_odom = true;
  }

  // if (cnt % IMU_PUB_PERIOD == 0 && !publish_imu) {
  //   imu_receiver.update();

  //   imu.stamp = now();
  //   imu.temperature = imu_receiver.temp;
  //   imu.accel_x = imu_receiver.ax;
  //   imu.accel_y = imu_receiver.ay;
  //   imu.accel_z = imu_receiver.az;
  //   imu.gyro_x = imu_receiver.gx;
  //   imu.gyro_y = imu_receiver.gy;
  //   imu.gyro_z = imu_receiver.gz;

  //   publish_imu = true;
  // }
}

int main() {
  setup();

  Ticker update_ticker;
  update_ticker.attach(&update, 0.01);

  while (true) {
    loop();
  }
}