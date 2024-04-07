import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Init rviz config path
    rviz_config_dir = os.path.join(
        get_package_share_directory('my_slam_toolbox_pkg'), 
        'config', 
        'config.rviz'
    )

    # Declare a launch argument for the map file path
    mapping_config = os.path.join(
        get_package_share_directory('my_slam_toolbox_pkg'),
        'config',
        'mapping.yaml'
    )

    return LaunchDescription([
    DeclareLaunchArgument(
        'use_sim_time',
        default_value="True", 
        description='Turn on/off sim time setting'
    ),

    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_2_foot_print',
        output='screen',
        emulate_tty=True,
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
    ),

    Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        arguments=['-d', rviz_config_dir]
    ),

    Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[mapping_config, 
        {'use_sim_time': LaunchConfiguration("use_sim_time")}]
    )

    ]) 