# Launch file for Gazebo
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='use_sim_time',
    ),
    DeclareLaunchArgument(
        'world_name', default_value='empty', description='Game field to load'
    ),
    DeclareLaunchArgument(
        'robot_name', default_value='mtt_robot', description='Robot name'
    ),
    DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Start rviz.',
    ),
    DeclareLaunchArgument(
        'run_on_start',
        default_value='true',
        choices=['true', 'false'],
        description='Run simulation on start.',
    ),
    DeclareLaunchArgument(
        'gui_config_file', default_value='', description='GUI config file.'
    ),
    DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Launch in headless mode (only gz server).',
    ),
    DeclareLaunchArgument(
        'verbose',
        default_value='false',
        choices=['true', 'false'],
        description='Launch in verbose mode.',
    ),
]


def simulation(
    world_name: str,
    gui_config: str = '',
    headless: bool = False,
    verbose: bool = False,
    run_on_start: bool = True,
):
    """Launch a simulation in Gazebo."""
    gz_args = []
    if gui_config != '':
        gz_args.append(f'--gui-config {gui_config}')
    if verbose:
        gz_args.append('-v 4')
    if run_on_start:
        gz_args.append('-r')
    if headless:
        gz_args.append('-s')

    if world_name.split('.')[-1] == 'sdf':
        gz_args.append(world_name)
    else:
        gz_args.append(f'{world_name}.sdf')

    pkg_ros_gz_gazebo = get_package_share_directory('ros_gz_sim')
    gz_gazebo_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_gazebo, 'launch', 'gz_sim.launch.py']
    )
    gz_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_gazebo_launch_path),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items(),
    )

    return gz_gazebo_launch


def launch_simulation(context: LaunchContext):
    """
    Return processes needed for launching the simulation.

    Simulator + Spawning Models + Bridges.
    """
    world_to_load = LaunchConfiguration('world_name').perform(context)
    gui_config_file = LaunchConfiguration('gui_config_file').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    headless = headless.lower() in ['true', 't', 'yes', 'y', '1']
    run_on_start = LaunchConfiguration('run_on_start').perform(context)
    run_on_start = run_on_start.lower() in ['true', 't', 'yes', 'y', '1']
    verbose = LaunchConfiguration('verbose').perform(context)
    verbose = verbose.lower() in ['true', 't', 'yes', 'y', '1']

    launch_processes = []
    launch_processes.append(
        simulation(world_to_load, gui_config_file, headless, verbose, run_on_start)
    )
    return launch_processes


def generate_launch_description():
    # Directories
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_simulator = get_package_share_directory('robot_simulator')
    pkg_robot_worlds = get_package_share_directory('robot_worlds')

    # set gazebo resource path
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_robot_worlds, 'worlds']),
            ':',
            os.path.join(get_package_prefix('robot_description'), 'share'),
            ':',
            PathJoinSubstitution([pkg_robot_worlds, 'models']),
            ':',
            PathJoinSubstitution([pkg_robot_worlds, 'meshes']),
        ],
    )

    # Paths
    robot_description_path = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'robot_rviz.launch.py']
    )
    robot_ros_gz_bridge_path = PathJoinSubstitution(
        [pkg_robot_simulator, 'launch', 'ros_gz_bridge.launch.py']
    )
    robot_spawn_path = PathJoinSubstitution(
        [pkg_robot_simulator, 'launch', 'spawn.launch.py']
    )

    # ------------------------------------------
    # ----------------- Launch -----------------
    # ------------------------------------------

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_path]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('use_gui', 'false'),
            ('vehicle', 'MTT_robot'),
        ],
    )

    robot_ros_gz_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_ros_gz_bridge_path]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world_name', LaunchConfiguration('world_name')),
            ('robot_name', LaunchConfiguration('robot_name')),
        ],
    )

    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_path]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('robot_name', LaunchConfiguration('robot_name')),
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(robot_description_launch)
    ld.add_action(robot_ros_gz_bridge_launch)
    ld.add_action(robot_spawn_launch)
    ld.add_action(
        OpaqueFunction(function=launch_simulation),
    )
    return ld
