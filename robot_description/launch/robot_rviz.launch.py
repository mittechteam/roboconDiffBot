# launch file for rviz2
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # package
    robot_description_pkg_prefix = get_package_share_directory('robot_description')

    # Paths
    rviz_config_path = robot_description_pkg_prefix + '/config/rviz_config.rviz'

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
            arguments=['-d', rviz_config_path]
        ),
    ])
