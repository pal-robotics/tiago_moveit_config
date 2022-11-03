moveit_controller_manager: moveit_ros_control_interface/Ros2ControlManager
moveit_simple_controller_manager:
  controller_names:
@[if has_arm]@
    - arm_controller
@[end if]@
    - torso_controller
    - head_controller
@[if end_effector == "pal-gripper" or end_effector == "schunk-wsg" or end_effector == "robotiq-2f-85" or end_effector == "robotiq-2f-140"]@
    - gripper_controller
@[end if]@
@[if end_effector == "pal-hey5"]@
    - hand_controller
@[end if]@
@[if has_arm]@
  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - arm_1_joint
      - arm_2_joint
      - arm_3_joint
      - arm_4_joint
      - arm_5_joint
      - arm_6_joint
      - arm_7_joint
@[end if]@
  torso_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - torso_lift_joint
  head_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - head_1_joint
      - head_2_joint
@[if end_effector == "pal-gripper"]@
  gripper_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_finger_joint
      - gripper_right_finger_joint
@[end if]@
@[if end_effector == "schunk-wsg" or end_effector == "robotiq-2f-85" or end_effector == "robotiq-2f-140"]@
  gripper_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_finger_joint
@[end if]@
@[if end_effector == "pal-hey5"]@
  hand_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - hand_index_joint
      - hand_thumb_joint
      - hand_mrl_joint
@[end if]@

trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01