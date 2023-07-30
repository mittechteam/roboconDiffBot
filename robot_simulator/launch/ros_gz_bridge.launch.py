# Copyright 2023 MTT
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Rahul Katiyar


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use sim time',
    ),
    DeclareLaunchArgument(
        'robot_name', default_value='mtt_robot', description='Robot name'
    ),
    DeclareLaunchArgument('world', default_value='default', description='World name'),
]


def generate_launch_description():
    # '    @  == a bidirectional bridge, \n'
    # '    [  == a bridge from Gazebo to ROS,\n'
    # '    ]  == a bridge from ROS to Gazebo.\n'
    # '  parameter_bridge [<topic@ROS2_type@Ign_type> ..] '

    # cmd_vel bridge
    ros__gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        namespace='',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.geometry_msgs.Twist',
            '/world/',
            LaunchConfiguration('world'),
            '/model/',
            LaunchConfiguration('robot_name'),
            '/joint_state',
            '@sensor_msgs/msg/JointState[gz.sensor_msgs.JointState',
            '/model/',
            LaunchConfiguration('robot_name'),
            '/pose',
            '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            (
                [
                    '/world/',
                    LaunchConfiguration('world'),
                    '/model/',
                    LaunchConfiguration('robot_name'),
                    '/joint_state',
                ],
                '/joint_states',
            ),
        ],
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ros__gazebo_bridge)
    return ld
