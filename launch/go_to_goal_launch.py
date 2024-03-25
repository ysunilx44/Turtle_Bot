from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='bowser_jr_object_follower',
        #     executable='get_nearest_object_node',
        #     name='get_nearest_object',
        # ),
        Node(
            package='bowser_jr_object_follower',
            executable='go_to_goal_node',
            name='go_to_goal',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),  # Dynamically finds the package path
                'launch', 
                'camera_robot.launch.py'  # Path relative to the package's share directory
            ]))
        )
    ])
