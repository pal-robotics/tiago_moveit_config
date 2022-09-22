import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_pal.arg_utils import read_launch_argument
from launch_pal.robot_utils import (get_arm,
                                    get_camera_model,
                                    get_end_effector,
                                    get_ft_sensor,
                                    get_laser_model,
                                    get_robot_name,
                                    get_wrist_model)
from launch_pal.substitutions import LoadFile
from ament_index_python.packages import get_package_share_directory

from tiago_description.tiago_launch_utils import get_tiago_hw_suffix
from launch_param_builder import load_xacro, load_yaml


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

    robot_description = {'robot_description': load_xacro(
        Path(os.path.join(
            get_package_share_directory('tiago_description'), 'robots', 'tiago.urdf.xacro')),
        {
            'arm': read_launch_argument('arm', context),
            'camera_model': read_launch_argument('camera_model', context),
            'end_effector': read_launch_argument('end_effector', context),
            'ft_sensor': read_launch_argument('ft_sensor', context),
            'laser_model': read_launch_argument('laser_model', context),
            'wrist_model': read_launch_argument('wrist_model', context),
        },
    )}

    robot_description_semantic_config = LoadFile(
        [get_package_share_directory("tiago_moveit_config"), "/config/srdf/tiago_",
         get_tiago_hw_suffix(arm=True, wrist_model=False,
                             end_effector=True, ft_sensor=True), ".srdf"]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        Path(os.path.join(
            get_package_share_directory('tiago_moveit_config'), 'config', 'kinematics_kdl.yaml'))
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        Path(os.path.join(
            get_package_share_directory('tiago_moveit_config'), 'config', 'ompl_planning.yaml'))
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = [
        get_package_share_directory(
            'tiago_moveit_config'), "/config/controllers/controllers_",
        get_tiago_hw_suffix(arm=True, wrist_model=False,
                            end_effector=True, ft_sensor=True), ".yaml"]

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    sim_time_parameters = {
        "use_sim_time": LaunchConfiguration("use_sim_time")
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_simple_controllers_path,
            planning_scene_monitor_parameters,
            sim_time_parameters,
        ],
    )

    return [run_move_group_node]


def generate_launch_description():
    # Command-line arguments
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use sim time"
    )

    ld = LaunchDescription()

    # Declare arguments
    # we use OpaqueFunction so the callbacks have access to the context
    ld.add_action(get_robot_name("tiago"))
    ld.add_action(OpaqueFunction(function=declare_args))
    ld.add_action(sim_time_arg)

    # Execute move_group node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
