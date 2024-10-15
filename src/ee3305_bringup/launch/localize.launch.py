from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    pkg_ee3305_bringup = FindPackageShare('ee3305_bringup')
    
    gazebo_launch_path = PathJoinSubstitution([pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py']),
    
    # tbot3 world
    launch_turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments = {
            'x_pose': '0',
            'y_pose': '0',
        }.items(),
    )
    ld.add_action(launch_turtlebot3_world)

    # nav2 bring_up
    yaml_map_file = PathJoinSubstitution([pkg_ee3305_bringup, 'maps', 'my_map.yaml'])
    yaml_params_file = PathJoinSubstitution([pkg_ee3305_bringup, 'params', 'nav2.yaml']) #from turtlebot3_navigation2 burger.yaml, changed 'robot_model_type: "nav2_amcl::DifferentialMotionModel"'
    launch_nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments = {
            'use_sim_time': 'True',
            'autostart': 'True',
            'map': yaml_map_file,
            'params_file': yaml_params_file
        }.items(),
    )
    ld.add_action(launch_nav2_bringup)
    
    # rviz 
    rviz_file = PathJoinSubstitution([pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'])
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )
    ld.add_action(node_rviz)

    return ld