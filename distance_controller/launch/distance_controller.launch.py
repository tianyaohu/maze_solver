from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([    
        DeclareLaunchArgument(
            'scene_num',  # Ensure this name matches the LaunchConfiguration name used below
            default_value='0',  # The default value should be a string
            description='Scene number: 0 for simulation, 1 for real'
        ),

        Node(
            package='distance_controller',
            executable='distance_controller_node',
            name='distance_controller_node',
            output='screen',
            parameters=[{'scene_num': LaunchConfiguration("scene_num")}])
    ])