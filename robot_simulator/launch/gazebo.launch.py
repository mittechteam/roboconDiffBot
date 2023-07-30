# Launch file for Gazebo
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGUMENTS = [
    DeclareLaunchArgument(
        "bridge",
        default_value="true",
        choices=["true", "false"],
        description="Use ros_gz_bridge",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="use_sim_time",
    ),
    DeclareLaunchArgument(
        "world", default_value="empty", description="Game field to load"
    ),
    DeclareLaunchArgument(
        "robot_name", default_value="mtt_robot", description="Robot name"
    ),
    DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Start rviz.",
    ),
    DeclareLaunchArgument(
        "spawn_dock",
        default_value="true",
        choices=["true", "false"],
        description="Spawn the standard dock model.",
    ),
]


def generate_launch_description():
    # Directories
    pkg_robot_description = get_package_share_directory("robot_description")
    pkg_robot_simulator = get_package_share_directory("robot_simulator")
    pkg_robot_worlds = get_package_share_directory("robot_worlds")
    pkg_ros_gz_gazebo = get_package_share_directory("ros_gz_sim")

    # set gazebo resource path
    gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([pkg_robot_worlds, "worlds"]),
            ":",
            os.path.join(get_package_prefix("robot_description"), "share"),
            ":",
            PathJoinSubstitution([pkg_robot_worlds, "models"]),
        ],
    )

    # Paths
    gz_gazebo_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_gazebo, "launch", "gz_sim.launch.py"]
    )
    robot_description_path = PathJoinSubstitution(
        [pkg_robot_description, "launch", "robot_rviz.launch.py"]
    )
    robot_ros_gz_bridge_path = PathJoinSubstitution(
        [pkg_robot_simulator, "launch", "ros_gz_bridge.launch.py"]
    )
    robot_spawn_path = PathJoinSubstitution(
        [pkg_robot_simulator, "launch", "spawn.launch.py"]
    )

    # ------------------------------------------
    # ----------------- Launch -----------------
    # ------------------------------------------
    gz_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_gazebo_launch_path),
        launch_arguments=[
            (
                "gz_args",
                [
                    LaunchConfiguration("world"),
                    ".sdf",
                ],
            )
        ],
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_path]),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("use_gui", "false"),
            ("vehicle", "MTT_robot"),
        ],
    )

    robot_ros_gz_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_ros_gz_bridge_path]),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("world", LaunchConfiguration("world")),
            ("robot_name", LaunchConfiguration("robot_name")),
        ],
    )

    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_path]),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("robot_name", LaunchConfiguration("robot_name")),
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(gz_gazebo_launch)
    ld.add_action(robot_description_launch)
    # ld.add_action(robot_ros_gz_bridge_launch)
    ld.add_action(robot_spawn_launch)
    return ld
