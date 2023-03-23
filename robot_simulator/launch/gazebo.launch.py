# Launch file for Gazebo
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Get the URDF file path
    urdf_file_name = 'robot_simulator.urdf.xml'
    urdf = get_package_share_directory('robot_description') + '/urdf/' + urdf_file_name

    # Get the Gazebo world file path
    world_file_name = 'world.world'
    world = get_package_share_directory('robot_simulator') + '/worlds/' + world_file_name

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so', world],
            parameters=[{'robot_description': urdf}]),
        Node(
            package='robot_simulator',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}]),
        Node(
            package='robot_simulator',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}]),
    ])
