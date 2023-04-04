# Launch file for Gazebo
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGUMENTS = [
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Use ros_ign_bridge'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Game field to load'),
    DeclareLaunchArgument('robot_name', default_value='mtt_robot',
                          description='Robot name'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
]

def generate_launch_description():

    # Directories
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_simulator = get_package_share_directory('robot_simulator')
    pkg_robot_worlds = get_package_share_directory('robot_worlds')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    # set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_robot_worlds, 'worlds']), ':',
            PathJoinSubstitution([pkg_robot_worlds, 'models'])
        ]
    )

    
    # Paths
    ign_gazebo_launch_path = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    robot_description_path = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'robot_rviz.launch.py'])
    robot_ros_ign_bridge_path = PathJoinSubstitution(
        [pkg_robot_simulator, 'launch', 'ros_ign_bridge.launch.py'])



    # ------------------------------------------
    # ----------------- Launch -----------------
    # ------------------------------------------
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ign_gazebo_launch_path),
        launch_arguments=[
            ('ign_args', [
                LaunchConfiguration('world'), '.sdf',
                ' -v 4',
                ])
        ]
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_path]),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    robot_ros_ign_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_ros_ign_bridge_path]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world', LaunchConfiguration('world')),
            ('robot_name', LaunchConfiguration('robot_name')),
        ]
    )


    # ------------------------------------------
    # ----------------- Nodes ------------------
    # ------------------------------------------
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', '0',
            '-y', '0',
            '-z', '0',
            '-Y', '0',
            '-topic', '/robot_description'],
        output='screen')
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gazebo_launch)
    ld.add_action(robot_description_launch)
    ld.add_action(robot_ros_ign_bridge_launch)
    # ld.add_action(spawn_robot)
    return ld
