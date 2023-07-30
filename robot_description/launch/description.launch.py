from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time',
    ),
    DeclareLaunchArgument(
        name='vehicle', default_value='MTT_robot', description='name of the vehicle'
    ),
    DeclareLaunchArgument(
        name='use_gui', default_value='false', description='use Joint state gui'
    ),
]


def generate_launch_description():
    # ------------------------------------------
    # --------------- Constants ----------------
    # ------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    robot_xacro_urdf_path = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro']
    )

    # ------------------------------------------
    # ----------------- Nodes ------------------
    # ------------------------------------------

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', robot_xacro_urdf_path]),
            }
        ],
    )

    # Joint state publisher
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        on_exit=Shutdown(),
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gui),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_pub)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(joint_state_publisher_node)
    return ld
