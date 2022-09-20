import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_pal.substitutions import LoadFile
from ament_index_python.packages import get_package_share_directory

from tiago_description.tiago_launch_utils import (get_tiago_hw_arguments,
                                                  get_tiago_hw_suffix,
                                                  generate_robot_description_action)

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # TIAGo description arguments
    tiago_args = get_tiago_hw_arguments(
        laser_model=True,
        arm=True,
        end_effector=True,
        ft_sensor=True,
        camera_model=True)

    # Read robot descripton from tiago_description
    robot_description = {
        "robot_description": generate_robot_description_action()}

    robot_description_semantic_config = LoadFile(
        [get_package_share_directory("tiago_moveit_config"), "/config/srdf/tiago_",
         get_tiago_hw_suffix(arm=True, wrist_model=False,
                             end_effector=True, ft_sensor=True), ".srdf"]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "tiago_moveit_config", "config/kinematics_kdl.yaml"
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
        "tiago_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        "tiago_moveit_config"), "config", "rviz")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
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

    ld = LaunchDescription()

    ld.add_action(*tiago_args)
    ld.add_action(rviz_node)

    return ld
