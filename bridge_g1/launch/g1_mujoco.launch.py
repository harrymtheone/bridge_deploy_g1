"""
Launch file for Unitree G1 robot in MuJoCo simulation

This launch file starts:
1. robot_state_publisher - Publishes robot model and TF transforms from joint states
2. mujoco_ros - MuJoCo simulator with G1 model
3. bridge_g1 - RL controller for G1 robot  
4. joy_node - Joystick for velocity commands
5. RViz for visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='g1_baseline',
        # default_value='g1_dreamwaq',
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
    
    # Load robot description from xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            g1_description_share,
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
    
    # RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare('bridge_g1'),
        'rviz',
        'g1.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
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
        headless_arg,
        render_rate_arg,
        robot_state_publisher_node,
        mujoco_node,
        joy_node,
        rviz_node,
        g1_node,
    ])

