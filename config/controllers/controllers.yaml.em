controller_manager_ns: controller_manager
controller_list:
@[if has_arm]@
  - name: arm_controller
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
  - name: torso_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - torso_lift_joint
  - name: head_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - head_1_joint
      - head_2_joint
@[if end_effector == "pal-gripper"]@
  - name: gripper_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_finger_joint
      - gripper_right_finger_joint
@[end if]@
@[if end_effector == "schunk-wsg" or end_effector == "robotiq-2f-85" or end_effector == "robotiq-2f-140"]@
  - name: gripper_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_finger_joint
@[end if]@
@[if end_effector == "pal-hey5"]@
  - name: hand_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - hand_index_joint
      - hand_thumb_joint
      - hand_mrl_joint
@[end if]@
