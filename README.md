# DDSM-210 ROS 2 Hardware Interface

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Distro: Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-green)](https://docs.ros.org/en/jazzy/index.html)

ROS 2 hardware interface for DDSM-210 servo motors with safety monitoring and thread-safe communication.

## Features

- **Configurable**: Runtime parameters for baud rate, timeout, and safety monitoring
- **Robust**: Automatic emergency stop on communication loss

## Installation

### Requirements

- ROS 2 Jazzy

### Setup

```bash
# Create workspace
export COLCON_WS=~/ros_ddsm_ws
mkdir -p $COLCON_WS/src
cd $COLCON_WS

# Clone repository
git clone https://github.com/AlessioMorale/ros_ddsm_servo.git src/ddsm-210

# Install dependencies and build
rosdep install --from-paths src --ignore-src -y
colcon build

# Source workspace
source install/setup.bash
```

## Configuration

Add to your URDF inside `<ros2_control>`:

```xml
<hardware>
  <plugin>ddsm210_hardware_interface/HardwareInterfaceDDSM210</plugin>
  <param name="device">/dev/ttyUSB0</param>
  <param name="serial_baud_rate">115200</param>
  <param name="communication_timeout_seconds">1.0</param>
  <param name="safety_check_period_ms">100</param>
</hardware>

<joint name="motor_1">
  <command_interface name="velocity"/>
  <state_interface name="velocity"/>
</joint>
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device` | string | `/dev/ttyUSB0` | Serial port device path |
| `serial_baud_rate` | int | `115200` | Baud rate (9600-921600) |
| `communication_timeout_seconds` | double | `1.0` | Timeout before emergency stop |
| `safety_check_period_ms` | int | `100` | Safety monitor check frequency |

## Usage

```bash
# Launch controller manager
ros2 launch ddsm210_hardware_interface demo.launch.py

# Send commands
ros2 topic pub /motor_1_velocity_controller/commands std_msgs/msg/Float64 "data: 50.0"

# Run tests
colcon test
```

## Troubleshooting

**Cannot open device**: Check permissions and connection:
```bash
ls -la /dev/ttyUSB*
sudo usermod -aG dialout $USER
```

**Motors stop unexpectedly**: 
- Check baud rate matches hardware configuration
- Increase `communication_timeout_seconds` if needed
- Verify serial cable connection

**Tests fail with conversion errors**: This is intentional (strict type checking). Ensure you're using C++17 with `-Werror=conversion` enabled.

## Architecture

The interface uses lock-free atomic timestamps for safe communication monitoring:
- Read/write operations store timestamps atomically (no mutex locks)
- Background safety monitor (10 Hz) checks for timeouts without contention
- Emergency stop triggered only when timeout detected

See `src/hardware_interface_ddsm210.cpp` for implementation details.

## Package Structure

```
ddsm-210/
├── ddsm210_driver/              # Serial communication driver
├── ddsm210_hardware_interface/  # ROS 2 hardware interface
│   ├── include/
│   ├── src/
│   └── test/                    # 42 unit tests
├── ddsm210_crsf_protocol/       # Protocol implementation
└── README.md
```

## License

MIT License - see [LICENSE](LICENSE) file
