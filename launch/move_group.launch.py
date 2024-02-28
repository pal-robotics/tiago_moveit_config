# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_pal.robot_utils import (get_arm,
                                    get_end_effector,
                                    get_ft_sensor,
                                    get_robot_name)
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix


def declare_args(context, *args, **kwargs):

    robot_name = read_launch_argument('robot_name', context)

    # TIAGo description arguments
    return [get_arm(robot_name),
            get_end_effector(robot_name),
            get_ft_sensor(robot_name)]


def launch_setup(context, *args, **kwargs):

    arm = read_launch_argument('arm', context)
    end_effector = read_launch_argument('end_effector', context)
    ft_sensor = read_launch_argument('ft_sensor', context)
    use_sensor_manager = read_launch_argument('use_sensor_manager', context)

    robot_description_semantic = ('config/srdf/tiago' + get_tiago_hw_suffix(
        arm=arm, end_effector=end_effector, ft_sensor=ft_sensor) + '.srdf')

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        'config/controllers/controllers' +
        get_tiago_hw_suffix(arm=arm, end_effector=end_effector, ft_sensor=ft_sensor) + '.yaml')

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    use_sim_time = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
    )

    if use_sensor_manager:
        # moveit_sensors path
        moveit_sensors_path = 'config/sensors_3d.yaml'
        moveit_config.sensors_3d(moveit_sensors_path)

    moveit_config.to_moveit_configs()

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            use_sim_time,
            moveit_config.to_dict(),
            {'publish_robot_description_semantic': True}
        ],
    )

    return [run_move_group_node]


def generate_launch_description():
    # Command-line arguments
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use sim time'
    )

    use_sensor_manager_arg = DeclareLaunchArgument(name='use_sensor_manager',
                                                   default_value='False',
                                                   choices=['True', 'False'],
                                                   description='Use moveit_sensor_manager \
                                                for octomap')

    ld = LaunchDescription()

    # Declare arguments
    # we use OpaqueFunction so the callbacks have access to the context
    ld.add_action(get_robot_name('tiago'))
    ld.add_action(OpaqueFunction(function=declare_args))
    ld.add_action(sim_time_arg)
    ld.add_action(use_sensor_manager_arg)

    # Execute move_group node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
