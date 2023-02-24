# launch file for rviz2
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

# include description.launch.py
from launch.actions import IncludeLaunchDescription
def generate_launch_description():
    
    # package
    robot_description_pkg_prefix = FindPackageShare('robot_description')

    # Launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_description_pkg_prefix, '/launch/description.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        robot_description_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', 'robot_description/config/rviz_config.rviz']
        ),
    ])
