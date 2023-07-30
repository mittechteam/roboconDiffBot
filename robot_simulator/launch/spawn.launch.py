# Launch file for Gazebo
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='mtt_robot',
                          description='Robot name'),
]

def generate_launch_description():
    # ------------------------------------------
    # ----------------- Nodes ------------------
    # ------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '0.0',
            # '-allow_renaming', 'true',
            '-topic', 'robot_description'],
        output='screen')
            
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot)
    return ld
