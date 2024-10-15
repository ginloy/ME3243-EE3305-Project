from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_ee3305_bringup = FindPackageShare('ee3305_bringup')

    gazebo_launch_path = PathJoinSubstitution([pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py']),
    params_path = PathJoinSubstitution([pkg_ee3305_bringup, 'params', 'project.yaml'])
    rviz_path = PathJoinSubstitution([pkg_ee3305_bringup, 'rviz', 'project.rviz'])
    map_path = PathJoinSubstitution([pkg_ee3305_bringup, 'maps', 'my_map.yaml'])
    # every time the params, rviz, or map path is replaced, the package has to be rebuilt.

    # tbot3 world
    launch_turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments = {
            'x_pose': '0',
            'y_pose': '0.5',
        }.items(),
    )
    ld.add_action(launch_turtlebot3_world)

    # behavior
    node_behavior = Node(
        package='ee3305_nav',
        executable='behavior',
        name='behavior',
        output='screen',
        parameters=[params_path],
    )
    ld.add_action(node_behavior)

    # map_server
    node_map_server = Node(
        package='ee3305_nav',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_path],
        arguments=[map_path],
    )
    ld.add_action(node_map_server)

    # estimator
    node_estimator = Node(
        package='ee3305_nav',
        executable='estimator',
        name='estimator',
        output='screen',
        parameters=[params_path],
    )
    ld.add_action(node_estimator)

    # planner
    node_planner = Node(
        package='ee3305_nav',
        executable='planner',
        name='planner',
        output='screen',
        parameters=[params_path],
    )
    ld.add_action(node_planner)
    
    # rviz 
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen'
    )
    ld.add_action(node_rviz)

# --prefix 'gdb -ex run --args'

    return ld