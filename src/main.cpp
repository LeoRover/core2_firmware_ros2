#include <mbed.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/trigger.h>

#include "diff_drive_lib/wheel_controller.hpp"

#include "configuration.hpp"
#include "microros/serial_transport.hpp"

static rcl_allocator_t default_allocator = rcl_get_default_allocator();
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rclc_parameter_server_t param_server;
static rcl_timer_t ping_timer, sync_timer;
static bool uros_agent_connected = false;
static bool ros_initialized = false;

static std_msgs__msg__Float32 battery;
static rcl_publisher_t battery_pub;
static bool publish_battery = false;

static UARTSerial uros_serial(RPI_SERIAL_TX, RPI_SERIAL_RX);

static void pingTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) uros_agent_connected = false;
}

static void syncTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  rmw_uros_sync_session(1000);
}

#define RCCHECK(fn) \
  if ((fn != RCL_RET_OK)) return false;

static bool initROS() {
  // Support
  RCCHECK(rclc_support_init(&support, 0, NULL, &default_allocator))

  // Node
  RCCHECK(rclc_node_init_default(&node, "firmware", "", &support))

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context,
                             15 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
                             &default_allocator))

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "~/battery"))

  // Timers
  RCCHECK(rclc_timer_init_default(&ping_timer, &support, RCL_MS_TO_NS(5000),
                                  pingTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &ping_timer))
  RCCHECK(rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(60000),
                                  syncTimerCallback))
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer))

  return true;
}

static void finiROS() {
  rclc_executor_fini(&executor);
  (void)!rcl_timer_fini(&ping_timer);
  (void)!rcl_timer_fini(&sync_timer);
  (void)!rcl_publisher_fini(&battery_pub, &node);
  (void)!rcl_node_fini(&node);
  rclc_support_fini(&support);
}

static void setup() {
  set_microros_serial_transports(&uros_serial);
}

static void loop() {
  // Try to connect to uros agent
  if (!ros_initialized && rmw_uros_ping_agent(1000, 1) == RMW_RET_OK) {
    ros_initialized = initROS();
    if (ros_initialized) {
      uros_agent_connected = true;
      (void)!rcl_timer_call(&sync_timer);
    } else
      finiROS();
  }

  if (!ros_initialized) return;

  // Handle agent disconnect
  if (!uros_agent_connected) {
    finiROS();
    ros_initialized = false;
    return;
  }

  rclc_executor_spin_some(&executor, 0);

  if (publish_battery) {
    (void)!rcl_publish(&battery_pub, &battery, NULL);
    publish_battery = false;
  }
}

static void update() {
  static uint32_t cnt = 0;
  ++cnt;

  if (!ros_initialized) return;

  if (cnt % 10 == 0 && !publish_battery) {
    battery.data = 0;
    publish_battery = true;
  }
}

int main() {
  Ticker update_ticker;
  update_ticker.attach(&update, 0.01);

  setup();

  while (true) {
    loop();
  }
}