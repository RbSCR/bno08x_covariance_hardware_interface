#!/usr/bin/env python3
"""
Launch file for the BNO08x IMU covariance hardware interface.

Starts the complete ros2_control stack for the BNO08x IMU, including:
- Robot state publisher for TF transforms
- Controller manager with the BNO08x SensorInterface hardware plugin
- State interfaces broadcaster
- IMU sensor covariance broadcaster publishing sensor_msgs/Imu to /imu_sensor_covariance_broadcaster/imu

Base usage:
    ros2 launch bno08x_covariance_hardware_interface bno08x_hal_covariance.launch.py

    See the declared_arguments below for the default values

Example (other) usage:
    <base-usage> i2c_bus:=1 i2c_addr:=4B
    <base-usage> axis_remap:=North-West-Up
    <base-usage> imu_rate:=150
    <base-usage> enable_magnetometer:=true
    <base-usage> enable_mock_mode:=true
    <base-usage> publish_tf:=false
    <base-usage> broadcast_magnetometer:=true
    <base-usage> publish_diagnostics:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number (e.g. 1 >> /dev/i2c-1)'
        ),
        DeclareLaunchArgument(
            'i2c_addr',
            default_value='4A',
            description='I2C device address in hex without 0x prefix (default: 4A = 0x4A, alternative: 4B)'
        ),
        DeclareLaunchArgument(
            'axis_remap',
            default_value='East-North-Up',
            description='BNO08X axis placement configuration: valid combination of North East South West Up Down'
                'in the format <xxx>-<xxx>-<xxx>'
                'See datasheet Figure 4-3  page 41 for the valid combinations.'
        ),
        DeclareLaunchArgument(
            'imu_rate',
            default_value='100',
            description='IMU sensor rate in Hz.'
                'See datasheet Figure 6-16  page 50.'
        ),
        DeclareLaunchArgument(
            'orientation_covariance_unreliable',
            default_value='0.0375',
            description='Orientation covariance when orientation-sensor status is unreliable'
        ),
        DeclareLaunchArgument(
            'orientation_covariance_low',
            default_value='0.0075',
            description='Orientation covariance when orientation-sensor status is low'
        ),
        DeclareLaunchArgument(
            'orientation_covariance_medium',
            default_value='0.0015',
            description='Orientation covariance when orientation-sensor status is medium'
        ),
        DeclareLaunchArgument(
            'orientation_covariance_high',
            default_value='0.00030',
            description='Orientation covariance when orientation-sensor status is high'
        ),
        DeclareLaunchArgument(
            'angular_velocity_covariance_unreliable',
            default_value='0.0625',
            description='Angular velocity covariance when angular-velocity-sensor status is unreliable'
        ),
        DeclareLaunchArgument(
            'angular_velocity_covariance_low',
            default_value='0.0125',
            description='Angular velocity covariance when angular-velocity-sensor status is low'
        ),
        DeclareLaunchArgument(
            'angular_velocity_covariance_medium',
            default_value='0.0025',
            description='Angular velocity covariance when angular-velocity-sensor status is medium'
        ),
        DeclareLaunchArgument(
            'angular_velocity_covariance_high',
            default_value='0.00050',
            description='Angular velocity covariance when angular-velocity-sensor status is high'
        ),
        DeclareLaunchArgument(
            'linear_acceleration_covariance_unreliable',
            default_value='5.0000',
            description='Linear acceleration covariance when linear_acceleration-sensor status is unreliable'
        ),
        DeclareLaunchArgument(
            'linear_acceleration_covariance_low',
            default_value='1.0000',
            description='Linear acceleration covariance when linear_acceleration-sensor status is low'
        ),
        DeclareLaunchArgument(
            'linear_acceleration_covariance_medium',
            default_value='0.2000',
            description='Linear acceleration covariance when linear_acceleration-sensor status is medium'
        ),
        DeclareLaunchArgument(
            'linear_acceleration_covariance_high',
            default_value='0.0400',
            description='Linear acceleration covariance when linear_acceleration-sensor status is high'
        ),
        DeclareLaunchArgument(
            'enable_magnetometer',
            default_value='false',
            description='Enable magnetometer in hardware_interface'
        ),
        DeclareLaunchArgument(
            'magnetometer_rate',
            default_value='100',
            description='Magnetometer rate in Hz.'
                'See datasheet Figure 6-16  page 50.'
        ),
        DeclareLaunchArgument(
            'magnetometer_covariance_unreliable',
            default_value='0.00000000125',
            description='Magnetometer covariance when magnetometer status is unreliable'
        ),
        DeclareLaunchArgument(
            'magnetometer_covariance_low',
            default_value='.00000000025',
            description='Magnetometer covariance when magnetometer status is low'
        ),
        DeclareLaunchArgument(
            'magnetometer_covariance_medium',
            default_value='0.000000000050',
            description='Magnetometer covariance when magnetometer status is medium'
        ),
        DeclareLaunchArgument(
            'magnetometer_covariance_high',
            default_value='.000000000010',
            description='Magnetometer covariance when magnetometer status is high'
        ),
        DeclareLaunchArgument(
            'enable_mock_mode',
            default_value='false',
            description='Use mock/simulation mode (no hardware required)'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description=(
                'Publish a dynamic world→base_link TF from IMU orientation for RViz visualization'
            ),
        ),
        DeclareLaunchArgument(
            # TODO(rbscr) diagnostics temporarily default disabled; awaiting ENHANCEMENT
            'publish_diagnostics',
            default_value='false',
            description=(
                'Run the bno08x_diagnostics companion node to publish sensor health '
                'and calibration status to /diagnostics at 1 Hz'
            ),
        ),
    ]

    i2c_bus = LaunchConfiguration('i2c_bus')
    i2c_addr = LaunchConfiguration('i2c_addr')
    axis_remap = LaunchConfiguration('axis_remap')
    imu_rate = LaunchConfiguration('imu_rate')
    orientation_covariance_unreliable = LaunchConfiguration('orientation_covariance_unreliable')
    orientation_covariance_low = LaunchConfiguration('orientation_covariance_low')
    orientation_covariance_medium = LaunchConfiguration('orientation_covariance_medium')
    orientation_covariance_high = LaunchConfiguration('orientation_covariance_high')
    angular_velocity_covariance_unreliable = LaunchConfiguration('angular_velocity_covariance_unreliable')
    angular_velocity_covariance_low = LaunchConfiguration('angular_velocity_covariance_low')
    angular_velocity_covariance_medium = LaunchConfiguration('angular_velocity_covariance_medium')
    angular_velocity_covariance_high = LaunchConfiguration('angular_velocity_covariance_high')
    linear_acceleration_covariance_unreliable = LaunchConfiguration('linear_acceleration_covariance_unreliable')
    linear_acceleration_covariance_low = LaunchConfiguration('linear_acceleration_covariance_low')
    linear_acceleration_covariance_medium = LaunchConfiguration('linear_acceleration_covariance_medium')
    linear_acceleration_covariance_high = LaunchConfiguration('linear_acceleration_covariance_high')

    enable_magnetometer = LaunchConfiguration('enable_magnetometer')
    magnetometer_rate = LaunchConfiguration('magnetometer_rate')
    magnetometer_covariance_unreliable = LaunchConfiguration('magnetometer_covariance_unreliable')
    magnetometer_covariance_low = LaunchConfiguration('magnetometer_covariance_low')
    magnetometer_covariance_medium = LaunchConfiguration('magnetometer_covariance_medium')
    magnetometer_covariance_high = LaunchConfiguration('magnetometer_covariance_high')

    enable_mock = LaunchConfiguration('enable_mock_mode')
    publish_tf = LaunchConfiguration('publish_tf')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('bno08x_covariance_hardware_interface'), 'config', 'bno08x_hal.urdf.xacro']
            ),
            ' ',
            'i2c_bus:=', i2c_bus,
            ' ',
            'i2c_addr:=', i2c_addr,
            ' ',
            'axis_remap:=', axis_remap,
            ' ',
            'imu_rate:=', imu_rate,
            ' ',
            'orientation_covariance_unreliable:=', orientation_covariance_unreliable,
            ' ',
            'orientation_covariance_low:=', orientation_covariance_low,
            ' ',
            'orientation_covariance_medium:=', orientation_covariance_medium,
            ' ',
            'orientation_covariance_high:=', orientation_covariance_high,
            ' ',
            'angular_velocity_covariance_unreliable:=', angular_velocity_covariance_unreliable,
            ' ',
            'angular_velocity_covariance_low:=', angular_velocity_covariance_low,
            ' ',
            'angular_velocity_covariance_medium:=', angular_velocity_covariance_medium,
            ' ',
            'angular_velocity_covariance_high:=', angular_velocity_covariance_high,
            ' ',
            'linear_acceleration_covariance_unreliable:=', linear_acceleration_covariance_unreliable,
            ' ',
            'linear_acceleration_covariance_low:=', linear_acceleration_covariance_low,
            ' ',
            'linear_acceleration_covariance_medium:=', linear_acceleration_covariance_medium,
            ' ',
            'linear_acceleration_covariance_high:=', linear_acceleration_covariance_high,
            ' ',
            'enable_magnetometer:=', enable_magnetometer,
            ' ',
            'magnetometer_rate:=', magnetometer_rate,
            ' ',
            'magnetometer_covariance_unreliable:=', magnetometer_covariance_unreliable,
            ' ',
            'magnetometer_covariance_low:=', magnetometer_covariance_low,
            ' ',
            'magnetometer_covariance_medium:=', magnetometer_covariance_medium,
            ' ',
            'magnetometer_covariance_high:=', magnetometer_covariance_high,
            ' ',
            'enable_mock_mode:=', enable_mock,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Controller configuration
    controller_config = PathJoinSubstitution(
        [FindPackageShare('bno08x_covariance_hardware_interface'), 'config', 'imu_hal_covariance_broadcaster.yaml']
    )

    # Controller manager (ros2_control_node)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[robot_description, controller_config],
    )

    # state interface broadcaster spawner
    state_interfaces_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['state_interfaces_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # IMU sensor covariance_broadcaster spawner
    imu_covariance_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_covariance_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Optional: relay IMU orientation to TF for RViz 3D visualization.
    # Set fixed frame to 'world' in RViz to see the sensor orientation animate.
    imu_hal_tf_broadcaster_node = Node(
        package='bno08x_covariance_hardware_interface',
        executable='imu_hal_tf_broadcaster',
        name='imu_hal_tf_broadcaster',
        output='screen',
        condition=IfCondition(publish_tf),
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            controller_manager_node,
            state_interfaces_spawner,
            imu_covariance_broadcaster_spawner,
            imu_hal_tf_broadcaster_node,
        ]
    )
