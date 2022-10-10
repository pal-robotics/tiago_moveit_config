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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_pal.robot_utils import (get_arm,
                                    get_camera_model,
                                    get_end_effector,
                                    get_ft_sensor,
                                    get_laser_model,
                                    get_robot_name,
                                    get_wrist_model)
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix


def declare_args(context, *args, **kwargs):

    robot_name = read_launch_argument('robot_name', context)

    # TIAGo description arguments
    return [get_arm(robot_name),
            get_camera_model(robot_name),
            get_end_effector(robot_name),
            get_ft_sensor(robot_name),
            get_laser_model(robot_name),
            get_wrist_model(robot_name)]


def launch_setup(context, *args, **kwargs):

    arm = read_launch_argument('arm', context)
    camera_model = read_launch_argument('camera_model', context)
    end_effector = read_launch_argument('end_effector', context)
    ft_sensor = read_launch_argument('ft_sensor', context)
    laser_model = read_launch_argument('laser_model', context)
    wrist_model = read_launch_argument('wrist_model', context)

    robot_description_path = os.path.join(
        get_package_share_directory('tiago_description'), 'robots', 'tiago.urdf.xacro')

    mappings = {
        'arm': arm,
        'camera_model': camera_model,
        'end_effector': end_effector,
        'ft_sensor': ft_sensor,
        'laser_model': laser_model,
        'wrist_model': wrist_model,
    }

    robot_description_semantic = ('config/srdf/tiago_' + get_tiago_hw_suffix(
        arm=arm, wrist_model=None, end_effector=end_effector, ft_sensor=ft_sensor) + '.srdf')

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        'config/controllers/controllers_' +
        get_tiago_hw_suffix(arm=arm, wrist_model=None,
                            end_effector=end_effector, ft_sensor=ft_sensor) + '.yaml')

    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description(file_path=robot_description_path, mappings=mappings)
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    use_sim_time = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'tiago_moveit_config'), 'config', 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            use_sim_time,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return [rviz_node]


def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use sim time'
    )

    ld = LaunchDescription()

    # Declare arguments
    # we use OpaqueFunction so the callbacks have access to the context
    ld.add_action(get_robot_name('tiago'))
    ld.add_action(OpaqueFunction(function=declare_args))
    ld.add_action(sim_time_arg)

    # Execute move_group node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
