# launch file for rviz2
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                            choices=['true', 'false'],
                            description='use_sim_time'),
    DeclareLaunchArgument(name='use_gui', default_value='false',
                        description='use Joint state gui'),
]

def generate_launch_description():

    # package
    robot_description_pkg_prefix = get_package_share_directory('robot_description')

    # Paths
    rviz_config_path = robot_description_pkg_prefix + '/config/rviz_config.rviz'

    # Launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_description_pkg_prefix, '/launch/description.launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_gui': LaunchConfiguration('use_gui'),
        }.items()
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', rviz_config_path]
        )


    ld =  LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description_launch)
    ld.add_action(rviz_node)
    return ld

