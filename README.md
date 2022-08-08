# core2_firmware_ros2

The firmware for the [Husarion CORE2] board running inside Leo Rover. 

The main functionalities include:
- velocity commands for the robot,
- velocity and PWM commands for individual wheels,
- battery voltage feedback,
- wheel states (position, velocity, PWM duty) feedback,
- odometry feedback (calculated from wheel encoders).

The project is written for [PlatformIO] and uses [Mbed OS] as the RTOS. It also uses [rcl+rclc] as a client library with [Micro XRCE-DDS] as a middleware to expose its functionalities on ROS topics, services and parameters.

For the documentation of the ROS API, visit [leo_fw] on the ROS wiki.

## Prerequisites
To build the project, all you'll need is [Visual Studio Code] with the [PlatformIO IDE] extension.

## Building
Open the project in [PlatformIO IDE], then run the `PlatformIO: Build` task.

## Flashing
### Using ST-Link programmer
Connect the ST-Link to the pins on [Husarion CORE2] debug pin header, then run the `PlatformIO: Upload` task.

You can also run the GDB debugger by running the `PIO Debug` launch configuration (or just clicking `F5`).

### Using RPi on Leo Rover
Upload the `.pio/build/core2/firmware.bin` to Leo Rover, then, on the robot, run:
```
ros2 run leo_fw flash firmware.bin
```

## Connecting
To expose the Micro-ROS node to the ROS2 network, you need to run the [Micro-ROS Agent] on RPi. Build the package using [colcon] and then run:
```
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/serial0 -b 460800
```

[Mbed OS]: https://os.mbed.com/mbed-os/
[Visual Studio Code]: https://code.visualstudio.com
[Husarion CORE2]: https://husarion.com/manuals/core2/
[leo_fw]: http://wiki.ros.org/leo_fw
[PlatformIO IDE]: https://platformio.org/platformio-ide
[PlatformIO]: https://docs.platformio.org/en/latest/what-is-platformio.html
[rcl+rclc]: https://github.com/ros2/rclc
[Micro XRCE-DDS]: https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/
[Micro-ROS Agent]: https://github.com/micro-ROS/micro-ROS-Agent
[colcon]: https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html