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

from gazebo_assets.bridges import bridges as gz_bridges
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    bridges = []

    # Get robot name from launch config in string format
    # robot_name_str = LaunchConfiguration('robot_name')

    bridges.append(gz_bridges.clock())
    bridges.append(gz_bridges.cmd_vel('mtt_robot'))

    ros__gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='world',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    )
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ros__gazebo_bridge)
    return ld
