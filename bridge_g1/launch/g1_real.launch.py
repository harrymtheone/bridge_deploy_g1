"""
Launch file for Unitree G1 real robot deployment

This launch file starts:
1. g1_real_node - RL controller connected to real G1 robot via Unitree SDK
2. robot_state_publisher - Publishes robot model and TF transforms
3. (Optional) joy_node - Joystick for velocity commands
4. (Optional) RViz for visualization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='g1_dreamwaq',
        description='Name of the model directory (contains config.yaml and model.onnx)'
    )

    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eno0',
        description='Network interface name for Unitree SDK communication (e.g., eth0, enp3s0)'
    )

    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Launch joystick node for velocity control'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # Load robot description from xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('g1_description'),
            'xacro',
            'unitree_G1.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher - publishes TF transforms from joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # G1 real robot controller node
    g1_real_node = Node(
        package='bridge_g1',
        executable='g1_real_node',
        name='g1_controller',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'network_interface': LaunchConfiguration('network_interface')
        }]
    )

    # Joystick node for control input (optional)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joy')),
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        model_name_arg,
        network_interface_arg,
        use_joy_arg,
        rviz_arg,
        robot_state_publisher_node,
        joy_node,
        rviz_node,
        g1_real_node,
    ])

