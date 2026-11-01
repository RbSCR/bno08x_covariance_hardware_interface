#!/usr/bin/env python3
"""
Launch file for the BNO08x IMU covariance hardware interface.

Starts the complete ros2_control stack for the BNO08x IMU, including:
- Robot state publisher for TF transforms
- Controller manager with the BNO08x SensorInterface hardware plugin
- State interfaces broadcaster
- IMU sensor covariance broadcaster publishing sensor_msgs/Imu to /imu_sensor_covariance_broadcaster/imu

Base usage:
    ros2 launch bno08x_covariance_hardware_interface bno08x_hal_covariance_fixedhwparams.launch.py

    See the declared_arguments below for the default values

Example (other) usage:
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
    ]

    enable_mock = LaunchConfiguration('enable_mock_mode')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_diagnostics = LaunchConfiguration('publish_diagnostics')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('bno08x_covariance_hardware_interface'), 'config', 'bno08x_hal_fixedhwparams.urdf.xacro']
            ),
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

    # IMU sensor covariance broadcaster spawner
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
