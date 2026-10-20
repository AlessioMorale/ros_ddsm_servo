![Slamming Works brand lockup](assets/slw-lockup.svg)

# DDSM-210 motor control

Slamming Works' ROS 2 hardware interface and serial driver for DDSM-210 servo motors.

## Demo

There is no motor demo video or GIF in this repository yet.

## Hardware

The driver communicates with DDSM-210 motors over a serial connection. The hardware interface is a `ros2_control` plugin; its device path, motor IDs and joint configuration belong in the robot's URDF/Xacro and controller configuration. A consolidated bill of materials and schematic are not included here.

## Build and run

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to ddsm210_hardware_interface
source install/setup.bash
```

Add the plugin to the robot's `<ros2_control>` block:

```xml
<hardware>
  <plugin>ddsm210_hardware_interface/HardwareInterfaceDDSM210</plugin>
  <param name="device">/dev/ttyUSB0</param>
  <param name="serial_baud_rate">115200</param>
  <param name="communication_timeout_seconds">1.0</param>
  <param name="safety_check_period_ms">100</param>
</hardware>
```

Start the controller manager using your robot's hardware description and controller configuration. When it is running, inspect the exposed interfaces with:

```bash
ros2 control list_hardware_interfaces
```

Run the package tests with:

```bash
colcon test --packages-select ddsm210_driver ddsm210_hardware_interface
colcon test-result --verbose
```

## Status

**Partly works.** The serial driver and `ros2_control` plugin are implemented and have test targets. This checkout does not include a motor bench-test recording or results log.

## License

MIT. See [LICENSE](LICENSE).
