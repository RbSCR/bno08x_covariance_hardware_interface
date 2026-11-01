# bno08x_covariance_hardware_interface

![Project Status](https://img.shields.io/badge/Status-Work%20in%20progress-orange)
![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20|%20Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![License](https://img.shields.io/github/license/rbscr/bno08x_rand?label=License)
[![ROS 2 Builder](https://github.com/RbSCR/bno08x_covariance_hardware_interface/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/RbSCR/bno08x_covariance_hardware_interface/actions/workflows/ros2-build.yml)

- [bno08x\_covariance\_hardware\_interface](#bno08x_covariance_hardware_interface)
  - [Overview](#overview)
  - [Features](#features)
  - [Hardware parameters and state interfaces](#hardware-parameters-and-state-interfaces)
    - [Hardware parameters](#hardware-parameters)
    - [State interfaces](#state-interfaces)
  - [Launch files and parameters](#launch-files-and-parameters)
    - [Launch files](#launch-files)
      - [bno08x\_hal\_covariance](#bno08x_hal_covariance)
      - [bno08x\_hal\_covariance\_magnetometer](#bno08x_hal_covariance_magnetometer)
      - [bno08x\_hal\_covariance\_fixedhwparams](#bno08x_hal_covariance_fixedhwparams)
    - [Launch parameters](#launch-parameters)
  - [Installation](#installation)
  - [Datasheet and documents](#datasheet-and-documents)
  - [Acknowledgements](#acknowledgements)
    - [Inspiration](#inspiration)
  - [License](#license)

## Overview

A ros2_control SensorInterface plugin for the CEVA BNO08x 9-DOF IMU's over I2C.

The BNO08x family (BNO085/BNO086) is a compact System in Package (SiP) with integrated accelerometer, gyroscope, magnetometer, and a 32-bit ARM® Cortex™-M0+ running CEVA's SH-2 firmware. It delivers real-time 3D orientation, heading, calibrated acceleration, and angular velocity, with on-board sensor fusion algorithms and calibration. It supports I2C, SPI, and UART interfaces for sensor data output.

>[!NOTE]
>This plugin only supports I2C.

## Features

- **10 IMU State Interfaces**: orientation quaternion {x, y, z, w}, angular velocity {x, y, z} (rad/s) and linear acceleration {x, y, z} (m/s²) — fully compatible with the ROS2 Control `imu_sensor_broadcaster`
- **4 IMU covariance State Interfaces**: covariance values for each sensor (orientation, angular velocity,linear accelaration) - compatible with `imu_sensor_covariance_broadcaster`
- **Axis Remapping**: 24 standard mounting orientations, configurable at launch, matching the datasheet Figure 4-3 page 41
- **Mock Mode**: Run the complete `ros2_control` lifecycle and publish zero/identity values without any hardware
- **3 Magnetometer State Interfaces** (optional): magnetic field {x, y, z} (Tesla) — fully compatible the ROS2 Control `magnetometer_broadcaster`. In mock mode publish zero values.
- **1 Magnetometer covariance State Interface** (optional): covariance value for magnetometer - compatible with `magnetometer_covariance_broadcaster`
- **TF Broadcasting** (optional): `imu_tf_broadcaster` relay node republishes the orientation quaternion as a dynamic `world → base_link` TF transform

The broadcasters `imu_sensor_covariance_broadcaster` and `magnetometer_covariance_broadcaster` are variants of the ROS2 Control standard broadcasters for an IMU sensor and a magnetometer. These variants use the covariance values in the state interface to determine the covariance (matrix diagonal) in the broadcasted messages.

## Hardware parameters and state interfaces

### Hardware parameters

| Parameter | Type | Description |
| --------- | ---- | ----------- |
| `i2c_bus` | `int` | I2C bus number ( plugin opens /dev/i2c-{x} ) |
| `i2c_address` | `string` | I2C address as hex without 0x prefix ( 4A = 0x4A ) |
| `axis_remap` | `string` | Sensor axis placement configuration, see datasheet Figure 4-3 page 41 |
| `imu_rate` | `int` | Rate at which to measure IMU data (Hz). Range: 0 - 400 Hz. see datasheet Figure 6-16 page 50 |
| `enable_magnetometer` | `bool` | Enable measuring of magnetic field data. |
| `magnetometer_rate` | `int` | Rate at which to measure magnetic field data (Hz). Range: 0 - 100 Hz. see datasheet Figure 6-16 page 50 |
| `orientation_covariance_unreliable` | `double` | Orientation covariance when orientation-sensor status is unreliable |
| `orientation_covariance_low` | `double` | Orientation covariance when orientation-sensor status is low |
| `orientation_covariance_medium` | `double` | Orientation covariance when orientation-sensor status is medium |
| `orientation_covariance_high` | `double` | Orientation covariance when orientation-sensor status is high |
| `angular_velocity_covariance_unreliable` | `double` | Angular velocity covariance when angular-velocity-sensor status is unreliable |
| `angular_velocity_covariance_low` | `double` | Angular velocity covariance when angular-velocity-sensor status is low |
| `angular_velocity_covariance_medium` | `double` | Angular velocity covariance when angular-velocity-sensor status is medium |
| `angular_velocity_covariance_high` | `double` | Angular velocity covariance when angular-velocity-sensor status is high |
| `linear_acceleration_covariance_unreliable` | `double` | Linear acceleration covariance when linear_acceleration-sensor status is unreliable |
| `linear_acceleration_covariance_low` | `double` | Linear acceleration covariance when linear_acceleration-sensor status is low |
| `linear_acceleration_covariance_medium` | `double` | Linear acceleration covariance when linear_acceleration-sensor status is medium |
| `linear_acceleration_covariance_high` | `double` | Linear acceleration covariance when linear_acceleration-sensor status is high |
| `magnetometer_covariance_unreliable` | `double` | Magnetometer covariance when magnetometer status is unreliable |
| `magnetometer_covariance_low` | `double` | Magnetometer covariance when magnetometer status is low |
| `magnetometer_covariance_medium` |`double` | Magnetometer covariance when magnetometer status is medium |
| `magnetometer_covariance_high` | `double` | Magnetometer covariance when magnetometer status is high |
| `enable_mock_mode` | `bool` | Skip I2C initialisation; publishes identity quaternion and zero velocity/acceleration/magnetic_field |

### State interfaces

| Interface | Unit | Notes |
| --------- | ---- | ----- |
| `orientation.x` | – | Quaternion X |
| `orientation.y` | – | Quaternion Y |
| `orientation.z` | – | Quaternion Z |
| `orientation.w` | – | Quaternion W |
| `angular_velocity.x` | rad/s | Gyroscope X |
| `angular_velocity.y` | rad/s | Gyroscope Y |
| `angular_velocity.z` | rad/s | Gyroscope Z |
| `linear_acceleration.x` | m/s² | Accelerometer X |
| `linear_acceleration.y` | m/s² | Accelerometer Y |
| `linear_acceleration.z` | m/s² | Accelerometer Z |
| `orientation_covariance` | – | Orientation covariance |
| `angular_velocity_covariance` | - | Gyroscope covariance |
| `linear_acceleration_covariance` | - | Accelerometer covariance |
| `magnetic_field.x` | Tesla | Magnetometer X - when magnetometer enabled |
| `magnetic_field.y` | Tesla | Magnetometer Y - when magnetometer enabled |
| `magnetic_field.z` | Tesla | Magnetometer Z - when magnetometer enabled |
| `magnetic_field_covariance` | - | Magnetometer covariance - when magnetometer enabled |

## Launch files and parameters

### Launch files

This package has 3 (example) launch-files:

- `bno08x_hal_covariance.launch.py`
- `bno08x_hal_covariance_magnetometer.launch.py`
- `bno08x_hal_covariance_fixedhwparams.launch.py`

#### bno08x_hal_covariance

This file launches the BNO08x with only the IMU enabled.
The related urdf-file is `bno08x_hal.urdf.xacro` and the related controller-config-file
is `imu_hal_covariance_broadcaster.yaml`.

The urdf- and config-file are not configured for the magnetometer.
The urdf-file contains defaults for the hardware parameters, which can be overruled by the
parameters in the launch-file.
The IMU broadcaster is the "new" (non standard ROIS2 Control) `imu_sensor_covariance_broadcaster`.

#### bno08x_hal_covariance_magnetometer

This file launches the BNO08x with the IMU and the magnetometer enabled.
The related urdf-file is `bno08x_hal_agnetometer.urdf.xacro` and the related controller-config-file
is `imu_hal_covariance_magnetometer_broadcaster.yaml`.

The urdf- and config-file are configured for the magnetometer.
The urdf-file contains defaults for the hardware parameters, which can be overruled by the
parameters in the launch-file.
The IMU broadcaster is the "new" `imu_sensor_covariance_broadcaster`, the magnetometer
broadcaster is the "new" (non standard ROIS2 Control) `magnetometer_covariance_broadcaster`.

#### bno08x_hal_covariance_fixedhwparams

This file launches the BNO08x with only the IMU enabled.
The related urdf-file is `bno08x_hal_fixedhwparams.urdf.xacro` and the related controller-config-file
is `imu_hal_covariance_broadcaster.yaml`.

The urdf- and config-file are not configured for the magnetometer.
The urdf-file contains the values of the hardware parameters, the related "launch-parameters" have
been removed from the launch-file.

This launch-file, urdf-file and controller-config-file can be used as a starting point for an
actual robot.
The IMU broadcaster is the "new" (non standard ROIS2 Control) `imu_sensor_covariance_broadcaster`.

Note: because this launch-file and urdf-file are also used in a test, the hardware
parameter `enable_moch_mode` does not have a 'fixed' value in the urdf-file, but is still used as
a parameter from the launch-file.

### Launch parameters

The launch parameters enable additional publishers/broadcasters.

The IMU measurements (orientation, angular velocity and linear acceleration) are always broadcasted using the imu broadcaster.

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `publish_tf` | `bool` | `"true"` | Publish a dynamic world→base_link TF from IMU orientation for RViz visualization |
| `broadcast_magnetometer` | `bool` | `"true"` | Broadcast magnetometer measurements using the magnetometer_broadcaster. To be usefull also set enable_magnetometer to true |

The hardware parameters (see the hardware parameter table) can also be used/set in the launch files `bno08x_hal_covariance.launch.py` and `bno08x_hal_covariance_magnetometer.launch.py` to overrule the defaults set in the related `urdf.xacro`-file.

## Installation

This plugin uses the library package `bno08x_sh2_libraries` to build the plugin and the packages `imu_sensor_covariance_broadcaster` and `magnetometer_covariance_broadcaster` to test the plugin.

Clone the repositories:

```bash
cd ~/ros_ws/src
git clone https://github.com/RbSCR/bno08x_sh2_driver.git
git clone https://github.com/RbSCR/imu_sensor_covariance_broadcaster
git clone https://github.com/RbSCR/magnetometer_covariance_broadcaster
git clone https://github.com/RbSCR/bno08x_covariance_hardware_interface.git
```

Install any missing dependencies:

```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -y
```

Build the package:

```bash
colcon build --packages-up-to bno08x_covariance_hardware_interface
```

## Datasheet and documents

| Name | Document number | Document revision | Date | By |
| ---- | --------------- | ----------------- | ---- | -- |
| [BNO080_085-Datasheet.pdf](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf) | 1000-3927 | 1.17 | July 24 2023 | CEVA hillcrestlabs |
| [SH-2-Reference-Manual.pdf](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf) | 1000-3625 | 1.9 | June 2021 | hillcrestlabs |
| [Sensor-Hub-Transport-Protocol-v1.7.pdf](https://cdn.sparkfun.com/assets/7/6/9/3/c/Sensor-Hub-Transport-Protocol-v1.7.pdf) | 1000-3535 | 1.7 | 02/16/2017 | hillcrestlabs |
| [Sensor-Calibration-Procedure-v1.1.pdf](https://cdn.sparkfun.com/assets/9/e/1/d/9/Sensor-Calibration-Procedure-v1.1.pdf) | 1000-4044 | 1.1 | February 2017 | hillcrestlabs |
| [BNO080-BNO085-Tare-Function-Usage-Guide.pdf](https://www.ceva-ip.com/wp-content/uploads/BNO080-BNO085-Tare-Function-Usage-Guide.pdf) | 1000-4045 | 1.3 | February 2023 | CEVA hillcrestlabs |

## Acknowledgements

This plugin (re)uses substantial parts of the code from the "BNO055 Hardware Interface" package by Aditya Kamath (see link below).

Through the `bno08x_sh2_libraries` package, this plugin also uses:

- the SH-2 protocol library provided by Hillcrest Labs; visit the official repository here: [SH-2 Protocol Library](https://github.com/ceva-dsp/sh2.git)
- and parts of the code from the [BNO08X ROS Driver](https://github.com/bnbhat/bno08x_ros2_driver) package by bnbhat.

### Inspiration

Inspiration for this package came from:

- the [BNO055 Hardware Interface](https://github.com/adityakamath/bno055_hardware_interface) package by Aditya Kamath.

## License

This package is licensed under the Apache License 2.0. You can find the full license text in the [LICENSE](./LICENSE) file of the repository.
