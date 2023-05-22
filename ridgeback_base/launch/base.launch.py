# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    physical_robot = LaunchConfiguration('physical_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    control_yaml_file = LaunchConfiguration('control_yaml_file')
    config = LaunchConfiguration('config')

    declare_physical_robot_cmd = DeclareLaunchArgument(
        'physical_robot',
        default_value='true',
        description='Physical robot if true, simulation if false')

    declare_control_yaml_cmd = DeclareLaunchArgument(
        'control_yaml_file',
        default_value=get_package_share_directory('ridgeback_control')+'/config/control.yaml',
        description='The config file for gazebo_ros2_control, only needed in simulation')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether or not to use simulation time')

    description_path = os.path.join(get_package_share_directory(
        'ridgeback_description'), 'launch', 'description.launch.py')

    declare_config_cmd = DeclareLaunchArgument(
        'config',
        default_value=os.getenv('RIDGEBACK_CONFIG', 'base'),
        description='get the ridgeback configuration')

    # Specify the actions
    description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_path),
        launch_arguments={
            'config': config,
            'physical_robot': physical_robot,
            'control_yaml_file': control_yaml_file}.items()
    )

    start_ridgeback_control = TimerAction(
            period=20.0,
            actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ridgeback_control'), 'launch', 'control.launch.py')),
            launch_arguments={
                'physical_robot': physical_robot,
                'control_yaml_file': control_yaml_file}.items())
                ]
        )

    start_teleop_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ridgeback_control'), 'launch', 'teleop.launch.py')),
    )

    ld = LaunchDescription()
    ld.add_action(declare_physical_robot_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_control_yaml_cmd)
    ld.add_action(declare_config_cmd)

    ld.add_action(description_cmd)
    ld.add_action(start_ridgeback_control)
    ld.add_action(start_teleop_control)

    return ld
