"""
Launch file for Unitree G1 robot in MuJoCo simulation

This launch file starts:
1. mujoco_ros - MuJoCo simulator with G1 model
2. bridge_g1 - RL controller for G1 robot  
3. robot_state_publisher - For RViz visualization
4. (Optional) RViz for visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='g1_dreamwaq',
        description='Name of the model directory (contains config.yaml and model.onnx)'
    )
    
    mjcf_model_arg = DeclareLaunchArgument(
        'mjcf_model',
        default_value='unitree_G1.xml',
        description='MJCF model file to use'
    )
    
    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate',
        default_value='1000.0',
        description='Simulation rate in Hz'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='500.0',
        description='Publish rate for sensor data in Hz'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run without MuJoCo GUI visualization'
    )

    render_rate_arg = DeclareLaunchArgument(
        'render_rate',
        default_value='60.0',
        description='Maximum rendering rate in FPS (0 for unlimited)'
    )
    
    # Get package share directories
    g1_description_share = FindPackageShare('g1_description')
    
    # Model path for MJCF (configurable)
    model_path = PathJoinSubstitution([
        g1_description_share, 'mjcf', LaunchConfiguration('mjcf_model')
    ])
    
    # MuJoCo simulation node (runs physics and publishes to ROS)
    mujoco_node = Node(
        package='mujoco_rospy',
        executable='mujoco_node',
        name='mujoco_sim',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'publish_rate': LaunchConfiguration('publish_rate'),
            'headless': LaunchConfiguration('headless'),
            'render_rate': LaunchConfiguration('render_rate')
        }]
    )
    
    # G1 controller node
    g1_node = Node(
        package='bridge_g1',
        executable='g1_mujoco_node',
        name='g1_controller',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name')
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
    
    # Joystick node for control input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )
    
    return LaunchDescription([
        model_name_arg,
        mjcf_model_arg,
        sim_rate_arg,
        publish_rate_arg,
        rviz_arg,
        headless_arg,
        render_rate_arg,
        mujoco_node,
        joy_node,
        rviz_node,
        g1_node,
    ])

