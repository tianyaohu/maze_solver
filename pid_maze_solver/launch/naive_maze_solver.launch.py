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

    return LaunchDescription([    
        DeclareLaunchArgument(
            'scene_num',
            default_value='0', 
            description='Scene number: 0 for simulation, 1 for real'
        ),

        DeclareLaunchArgument(
            'reverse_solve',
            default_value='false', 
            description='If true, solve maze from end to start; if false, solve from start to end'
        ),

        DeclareLaunchArgument(
            'topic_pose',
            default_value='odometry/filtered', 
            description='the topic to subscribe to for pose update, the message must contain type geometry_msgs/Pose pose'
        ),

        #Simulation
        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(empty_world_launch_file)
        # ),

        Node(
            package='pid_maze_solver',
            executable='naive_maze_solver_node',
            name='naive_maze_solver_node',
            output='screen',
            parameters=[{
                'scene_num': LaunchConfiguration("scene_num"),
                'reverse_solve': LaunchConfiguration("reverse_solve")  
            }])
    ])