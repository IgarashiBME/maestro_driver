# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 (Jazzy) ament_python package that bridges `/rc_pwm` topic commands to a Pololu Maestro servo controller over USB serial. Single-node package using pyserial and the Maestro Compact Protocol.

## Build & Test Commands

```bash
# Build (from workspace root)
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select maestro_driver

# Source after build
source ~/ros2_ws/install/setup.bash

# Run the node
ros2 run maestro_driver maestro_driver_node

# Run with parameter overrides
ros2 run maestro_driver maestro_driver_node --ros-args -p serial_port:=/dev/ttyACM1 -p failsafe_timeout:=1.0

# Run tests
cd ~/ros2_ws && colcon test --packages-select maestro_driver && colcon test-result --verbose
```

## Architecture

Single node (`MaestroDriverNode`) in `maestro_driver/maestro_driver_node.py`. Entry point: `main()`.

**Data flow:** `/rc_pwm` (UInt16MultiArray, BestEffort QoS) → clamp to [pwm_min, pwm_max] → Maestro Set Target command (0x84) over USB serial → servo controller

**Maestro protocol:** PWM μs × 4 = quarter-microseconds target. Encoded as two 7-bit bytes: `[0x84, channel, target & 0x7F, (target >> 7) & 0x7F]`

**Key behaviors:**
- Failsafe timer sends neutral PWM (1500μs) to both channels when `/rc_pwm` stops arriving (default 0.5s timeout)
- Serial port auto-reconnects on disconnect or startup failure (1s retry timer)
- Node starts in failsafe state; recovers when first valid message arrives
- Shutdown sends failsafe before closing serial port

## Conventions

- Language: Python with type hints
- All public/private methods have docstrings
- ROS2 logging uses `get_logger()` with `throttle_duration_sec` for repeated messages
- QoS: BestEffort, depth=1 (real-time control pattern)
- Linting: ament_flake8, ament_pep257, ament_copyright (standard ROS2 test suite in `test/`)
