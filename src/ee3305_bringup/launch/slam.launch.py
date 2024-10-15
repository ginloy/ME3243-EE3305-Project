from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_turtlebot3_cartographer = FindPackageShare('turtlebot3_cartographer')
    
    gazebo_launch_path = PathJoinSubstitution([pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py']),
    cartographer_launch_path = PathJoinSubstitution([pkg_turtlebot3_cartographer, 'launch', 'cartographer.launch.py']),

    # tbot3 world
    launch_turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments = {
            'x_pose': '0',
            'y_pose': '0',
        }.items(),
    )
    ld.add_action(launch_turtlebot3_world)

    # cartographer (slam)
    launch_turtlebot3_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_launch_path),
        launch_arguments = {
            'use_sim_time': 'True',
        }.items(),
    )
    ld.add_action(launch_turtlebot3_cartographer)

    return ld