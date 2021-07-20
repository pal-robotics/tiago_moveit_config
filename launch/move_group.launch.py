import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_pal.substitutions import LoadFile
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

from tiago_description.tiago_launch_utils import (get_tiago_hw_arguments,
                                                  get_tiago_hw_suffix,
                                                  generate_robot_description_action)

def load_yaml(package_name, file_path):
    package_path=get_package_share_directory(package_name)
    absolute_file_path=os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # TIAGo description arguments
    tiago_args=get_tiago_hw_arguments(
        laser_model=True,
        arm=True,
        end_effector=True,
        ft_sensor=True)

    # Command-line arguments
    sim_time_arg=DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use sim time"
    )

    # Read robot descripton from tiago_description
    robot_description={
        "robot_description": generate_robot_description_action()}

    robot_description_semantic_config=LoadFile(
        [get_package_share_directory("tiago_moveit_config"), "/config/srdf/tiago_",
        get_tiago_hw_suffix(arm=True, wrist_model=False,
                            end_effector=True, ft_sensor=True), ".srdf"]
    )
    robot_description_semantic={
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml=load_yaml(
        "tiago_moveit_config", "config/kinematics_kdl.yaml"
    )

    # Planning Functionality
    ompl_planning_pipeline_config={
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml=load_yaml(
        "tiago_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = [
        get_package_share_directory('tiago_moveit_config'), "/config/controllers/controllers_",
        get_tiago_hw_suffix(arm=True, wrist_model=False,
                            end_effector=True, ft_sensor=True), ".yaml"]

    trajectory_execution={
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters={
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    sim_time_parameters={
        "use_sim_time": LaunchConfiguration("use_sim_time")
    }

    # Start the actual move_group node/action server
    run_move_group_node=Node(
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

    # RViz
    tutorial_mode=LaunchConfiguration("rviz_tutorial")
    rviz_base=os.path.join(get_package_share_directory(
        "tiago_moveit_config"), "config", "rviz")
    rviz_full_config=os.path.join(rviz_base, "moveit.rviz")
    rviz_node=Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    return LaunchDescription(
        [
            *tiago_args,
            sim_time_arg,
            rviz_node,
            run_move_group_node,
        ]
    )
