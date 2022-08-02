#pragma once

// The period (in number of calls to the update() function) at which the battery
// voltage is probed
const uint8_t BATTERY_PROBE_PERIOD = 10;

// Number of battery voltage readings to average
const uint32_t BATTERY_BUFFER_SIZE = 300;

// The period (in milliseconds) between calls to the update() function
const uint16_t UPDATE_PERIOD = 10;

// The periods (in number of calls to the update() function) at which different
// data is publihed on the ROS topics
const uint8_t BATTERY_PUB_PERIOD = 10;