from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the path to the other package's launch file
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

        Node(
            package='pid_maze_solver',
            executable='naive_maze_solver_node',
            name='naive_maze_solver_node',
            output='screen',
            parameters=[{
                'scene_num': LaunchConfiguration("scene_num"),
                'reverse_solve': LaunchConfiguration("reverse_solve"),
                'topic_pose': LaunchConfiguration("topic_pose")   
            }])
    ])