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
            executable='identify_sign_node',
            name='identify_sign',
        ),
        Node(
            package='bowser_jr_object_follower',
            executable='get_dist_node',
            name='get_dist',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),  # Dynamically finds the package path
                'launch', 
                'camera_robot.launch.py'  # Path relative to the package's share directory
            ]))
        )
    ])
