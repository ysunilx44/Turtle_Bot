from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bowser_jr_object_follower',
            executable='find_object_node',
            name='find_object',
        ),
        Node(
            package='bowser_jr_object_follower',
            executable='get_object_range_node',
            name='get_object_range',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),  # Dynamically finds the package path
                'launch', 
                'camera_robot.launch.py'  # Path relative to the package's share directory
            ]))
        )
    ])
