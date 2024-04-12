import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():
    nav2_yaml = os.path.join(get_package_share_directory('localization'), 'config', 'real_amcl.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('my_slam_toolbox_pkg'), 'config', 'config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        arguments=['-d', rviz_config_dir]
    )

    map_static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_2_foot_print',
        output='screen',
        emulate_tty=True,
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
    )

    

    # laser_static_tf_pub = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_base_link_2_foot_print',
    #     output='screen',
    #     emulate_tty=True,
    #     arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
    # )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")},
                    {'yaml_filename':PathJoinSubstitution([FindPackageShare("localization"), 'map', LaunchConfiguration("map_file")])}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # Set up the event handlers to control the start-up sequence
    # rviz (continuous run) 
    # tf_pub -> map_server_node -> amcl_node -> lifecycle_manager_node
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value="real_map.yaml", 
            description='name of map_file within localization/map'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="False", 
            description='Turn on/off sim time setting'
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=map_static_tf_pub,
                on_start=[                    
                    TimerAction(
                        period=5.0, # seconds
                        actions=[map_server_node],
                    )],  
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_server_node,
                on_start=[amcl_node],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=amcl_node,
                on_start=[lifecycle_manager_node],
            )
        ),

        rviz_node, 
        map_static_tf_pub,
        # laser_static_tf_pub

    ])
