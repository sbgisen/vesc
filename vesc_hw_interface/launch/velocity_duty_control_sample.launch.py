#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2023 SoftBank Corp.
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
#

import pathlib

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:

    vesc_pkg = FindPackageShare('vesc_hw_interface').find('vesc_hw_interface')
    doc = xacro.process_file(LaunchConfiguration('model').perform(context))
    robot_description = {"robot_description": doc.toprettyxml(indent='  ')}

    robot_controllers = [vesc_pkg, '/config/velocity_duty_sample.yaml']

    control_node = Node(package="controller_manager",
                        executable="ros2_control_node",
                        parameters=[robot_description, robot_controllers],
                        output="both")

    controllers = GroupAction(actions=[Node(package='controller_manager',
                                            executable='spawner',
                                            output='both',
                                            arguments=["--controller-manager", "controller_manager",
                                                       'joint_state_broadcaster']),
                                       Node(package='controller_manager',
                                            executable='spawner',
                                            output='both',
                                            arguments=["--controller-manager", "controller_manager",
                                                       'joint_velocity_controller'])])

    return [control_node, controllers]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    vesc_pkg = pathlib.Path(FindPackageShare('vesc_hw_interface').find('vesc_hw_interface'))
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=str(vesc_pkg / 'launch/velocity_duty_test.ros2_control.xacro'))

    return LaunchDescription([
        model_arg,
        OpaqueFunction(function=launch_setup)
    ])
