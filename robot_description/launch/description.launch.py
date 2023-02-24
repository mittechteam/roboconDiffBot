from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time'),

    robot_urdf_path = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro']
    )

    
    return LaunchDescription([

        DeclareLaunchArgument(
            name='robot_urdf', 
            default_value=robot_urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            name="vehicle",
            default_value="'MTT_robot'",
            description="name of the vehicle"
        ),

        # Robot state publisher   
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('robot_urdf')]), 
                'use_sim_time': use_sim_time
            }],
            condition=IfCondition(PythonExpression([LaunchConfiguration("vehicle"), " == 'MTT_robot'"]))
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_gui': True,
                'use_sim_time': use_sim_time
            }],
            condition=IfCondition(PythonExpression([LaunchConfiguration("vehicle"), " == 'MTT_robot'"]))
        ),
    
    ])
