import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the other package's launch file
    sim_pkg = get_package_share_directory('rosbot_xl_gazebo')
    empty_world_launch_file = os.path.join(sim_pkg, 'launch', 'empty_simulation.launch.py')
    return LaunchDescription([    
        DeclareLaunchArgument(
            'scene_num',
            default_value='0', 
            description='Scene number: 0 for simulation, 1 for real'
        ),

        #Simulation
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(empty_world_launch_file)
        ),

        Node(
            package='distance_controller',
            executable='distance_controller_node',
            name='distance_controller_node',
            output='screen',
            parameters=[{'scene_num': LaunchConfiguration("scene_num")}])
    ])