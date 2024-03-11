^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.15 (2024-03-11)
-------------------
* Merge branch 'velocity_interface' into 'erbium-devel'
  Velocity interface
  See merge request robots/tiago_moveit_config!71
* add all the velocity controllers params to the yaml file of each gripper
* joint velocity limit modified for the joint 6
* Contributors: Sai Kishor Kothakota, ileniaperrella

1.2.14 (2024-02-28)
-------------------
* Merge branch 'update-sensor-padding' into 'erbium-devel'
  Update sensor padding
  See merge request robots/tiago_moveit_config!73
* Update sensor padding
* Contributors: David ter Kuile, davidterkuile

1.2.13 (2024-02-28)
-------------------
* Merge branch 'update-move-group' into 'erbium-devel'
  Update move group
  See merge request robots/tiago_moveit_config!72
* Update dependency moveit_chomp_optimizer_adapter
* Reset ompl default settings
* Rename to ompl_chomp
* Update move_group
* Contributors: David ter Kuile, davidterkuile

1.2.12 (2023-12-21)
-------------------
* Update advanced_grasping sensors
* Contributors: David ter Kuile

1.2.11 (2023-10-04)
-------------------
* Merge branch 'virtual-joint-issue' into 'erbium-devel'
  Removed virtual joint for grasp candidates compatibility
  See merge request robots/tiago_moveit_config!62
* Update octomap properties of advanced grasping
* Removed virtual joint for grasp candidates compatibility
* Contributors: David ter Kuile, davidterkuile, sergiacosta

1.2.10 (2023-07-11)
-------------------
* Merge branch 'padding-update' into 'erbium-devel'
  Update deprecated namespace of padding.yaml
  See merge request robots/tiago_moveit_config!59
* Add arm_padding only if robot has arm
* Update deprecated namespace of padding.yaml
* Merge branch 'mlu/fix/update-script' into 'erbium-devel'
  Enable pipefail in update script
  See merge request robots/tiago_moveit_config!57
* Enable pipefile in update script
* Contributors: David ter Kuile, Mathias Lüdtke, Sai Kishor Kothakota

1.2.9 (2023-06-20)
------------------
* Merge branch 'mlu/fix/epick-srdfs' into 'erbium-devel'
  Fix old epick disable_collisions/*.srdf.xacro
  See merge request robots/tiago_moveit_config!56
* Add missing links from no-ee to epick SRDFs
* Fix old epick disable_collisions/*.srdf.xacro
  with-arm got renamed to no-ee in the meantime
* Contributors: Mathias Lüdtke, Sai Kishor Kothakota

1.2.8 (2023-04-11)
------------------
* Merge branch 'add-odom-virtual-joint' into 'erbium-devel'
  Add odom virtual joint
  See merge request robots/tiago_moveit_config!55
* Updated files using update.sh script
* Added virtual joint to srdf.xacro file
* Contributors: saikishor, sergiacosta

1.2.7 (2023-04-11)
------------------
* Merge branch 'add-epick-srdf' into 'erbium-devel'
  Updated srdf files with epick gripper
  See merge request robots/tiago_moveit_config!54
* Updated epick gripper dependent files using update.sh
* Contributors: saikishor, sergiacosta

1.2.6 (2023-04-03)
------------------
* Merge branch 'add-epick-gripper-xacro' into 'erbium-devel'
  Added eqick gripper xacro files
  See merge request robots/tiago_moveit_config!53
* Added eqick gripper xacro files for automatic srdf files generation
* Contributors: saikishor, sergiacosta

1.2.5 (2023-02-28)
------------------
* Merge branch 'fix-gallium-dependencies' into 'erbium-devel'
  Add noetic condition on pal-moveit-plugins in package.xml
  See merge request robots/tiago_moveit_config!51
* Add noetic condition on pal-moveit-plugins in package.xml
* Contributors: David ter Kuile, Jordan Palacios

1.2.4 (2022-12-13)
------------------
* Merge branch 'extra-moveit-capabilities' into 'erbium-devel'
  Extra moveit capabilities
  See merge request robots/tiago_moveit_config!47
* Add arguments for advanced grasping
* Add sensor manager param
* Add dependency on pal_moveit_plugins
* set capability default back to empty
* Add capability loader plugin
* Add extra capabilities
* If no arm don't load padding of arm
* Add link padding to arm-link-5
* Contributors: David ter Kuile, saikishor

1.2.3 (2022-08-01)
------------------
* Merge branch 'gallium-fix' into 'erbium-devel'
  Add default_velocity_scaling_factor to avoid slow movements
  See merge request robots/tiago_moveit_config!32
* Add default_velocity_scaling_factor to avoid slow movements
* Contributors: David ter Kuile, Jordan Palacios

1.2.2 (2022-07-19)
------------------
* Merge branch 'mlu/fix/move-scripts-out' into 'erbium-devel'
  Use pal_moveit_config_generator
  See merge request robots/tiago_moveit_config!36
* Show warning if pal_moveit_config_generator is missing
* Clean-up installed files
* Use pal_moveit_config_generator
* Contributors: Mathias Lüdtke, saikishor

1.2.1 (2022-07-13)
------------------
* Merge branch 'mlu/fix/hey5-srdf' into 'erbium-devel'
  Merge missing disable collision pairs
  See merge request robots/tiago_moveit_config!34
* Merge missing disable collision pairs
* Fix generate_srdf.sh to lazy-load descriptions
* Contributors: Mathias Lüdtke, saikishor

1.2.0 (2022-05-03)
------------------
* Merge branch 'no-end-effector-bugfix' into 'erbium-devel'
  No end effector bugfix
  See merge request robots/tiago_moveit_config!31
* file_suffix consistency
* remove redundant files
* update
* fix empty target matrxi bug
* Fix generation of  empty collision matrices
* wip for while loop
* Updated srdf with new update.sh script
* Add srdf scirtps from mathias
* small update
* Update missing ft_sensor links in srdf
* some changes
* update eval function with no end-effector
* fix empty target matrxi bug
* Fix generation of  empty collision matrices
* wip for while loop
* Updated srdf with new update.sh script
* Add srdf scirtps from mathias
* small update
* Update missing ft_sensor links in srdf
* some changes
* update eval function with no end-effector
* Contributors: David ter Kuile, mathiasluedtke, saikishor

1.1.4 (2022-03-18)
------------------
* Merge branch 'mlu/feature/omni-base-srdf' into 'erbium-devel'
  Add SRDFs for omni_base and epick
  See merge request robots/tiago_moveit_config!30
* Add vacuum joint to end effector group of Robotiq EPick
* Remove gripper_controller from the controller_manager list for epick
* Add generated config files for epick controller
* Add config files for epick gripper
* Add srdf xacro and gernerated srdf for robotiq-epick gripper
* Add SRDFs for omni_base
* Merge branch 'mlu/fix/test-launch-files' into 'erbium-devel'
  Fix URDF loading in planning_context.launch
  See merge request robots/tiago_moveit_config!28
* Fix demo.launch
  delegate URDF/SRDF loading to move_group.launch
* Fix URDF loading in planning_context.launch
* Contributors: Mathias Lüdtke, saikishor, thomaspeyrucain

1.1.3 (2021-12-02)
------------------
* Merge branch 'add_base_type' into 'erbium-devel'
  Add base_type argument to the moveit launch files
  Closes tiago_dual_moveit_config#1
  See merge request robots/tiago_moveit_config!24
* Add base_type argument to the moveit launch files
* Contributors: Sai Kishor Kothakota, victor

1.1.2 (2021-11-09)
------------------
* Merge branch 'update_srdf_format' into 'erbium-devel'
  Update srdf, remove initial message
  See merge request robots/tiago_moveit_config!23
* Update srdf, remove initial message
* Contributors: Jordan Palacios, cescfolch

1.1.1 (2021-05-06)
------------------

1.1.0 (2021-05-06)
------------------
* Merge branch 'robotiq_gripper' into 'erbium-devel'
  Robotiq gripper
  See merge request robots/tiago_moveit_config!22
* update the SRDF with the missing FT links
* Update the SRDF configuration
* update the robotiq end effector naming
* initial commit of robotiq 85 and 140 moveit config of TIAGo
* Add README and update setup assistant xacro file name
* Contributors: Sai Kishor Kothakota, Victor Lopez, saikishor

1.0.6 (2020-10-01)
------------------
* Merge branch 'hey5_marker' into 'erbium-devel'
  Hey5 marker
  See merge request robots/tiago_moveit_config!21
* Hey5 marker
* Contributors: Adria Roig, victor

1.0.5 (2020-06-09)
------------------
* Add arm_5 wrist ignore collisions
* Contributors: Victor Lopez

1.0.4 (2020-04-21)
------------------
* Merge branch 'custom-ee' into 'erbium-devel'
  Allow using custom end-effector
  See merge request robots/tiago_moveit_config!19
* Allow using custom end-effector
* Contributors: davidfernandez, victor

1.0.3 (2020-02-06)
------------------
* Merge branch 'move_group_capability' into 'erbium-devel'
  send capabilities through args
  See merge request robots/tiago_moveit_config!18
* send capabilities through args
* Contributors: Victor Lopez, YueErro

1.0.2 (2019-08-22)
------------------
* Add missing multi argument
* Decrease segment size for validation
* Contributors: Victor Lopez

1.0.1 (2018-12-19)
------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Added autogenerated srdf
  See merge request robots/tiago_moveit_config!16
* Restore old camera parameter
* Refactor controllers files
* Refactor joint limits and srdf
* Added autogenerated srdf
* Contributors: Victor Lopez

1.0.0 (2018-12-19)
------------------

0.0.22 (2018-07-30)
-------------------
* Merge branch 'fix-simulation-warnings' into 'cobalt-devel'
  fix deprecated namespace
  See merge request robots/tiago_moveit_config!15
* fix deprecated namespace
* fix demo mode by adding missing argument
  You hacked multi-robot support into a generated moveit configuration
  but didn't test "roslaunch tiago_moveit_config demo.launch".
  I agree that gazebo support is better than the demo mode, but
  it can be very useful to test MoveIt-based code without controlling.
* Contributors: Jordi Pages, Victor Lopez, v4hn

0.0.21 (2018-03-28)
-------------------
* Merge branch 'disable-sonar-collision' into 'cobalt-devel'
  Disable sonar collision with base_link
  See merge request robots/tiago_moveit_config!14
* Disable sonar collision with base_link
* Contributors: Victor Lopez, davidfernandez

0.0.20 (2018-03-26)
-------------------
* Merge branch 'recover-chessboard-tiago' into 'cobalt-devel'
  Disable collision between arm 7 and chessboard
  See merge request robots/tiago_moveit_config!13
* Disable collision between arm 7 and chessboard
* Contributors: Jordi Pages, Victor Lopez

0.0.19 (2018-01-24)
-------------------
* add config files for schunk and some renamings
* Contributors: Jordi Pages

0.0.18 (2017-11-03)
-------------------
* Change the topic and the max_range for the octomap parameters
* Contributors: AleDF, Jordi Pages

0.0.17 (2017-05-16)
-------------------
* Merge branch 'octomap_track_ik' into 'cobalt-devel'
  merge_problems_with david
  See merge request !11
* merge_problems_with david
* Merge branch 'iron-configuration' into 'cobalt-devel'
  Add configuration for Tiago Iron
  See merge request !10
* Merge branch 'octomap_track_ik' into 'cobalt-devel'
  octomap & track ik solver for MoveIt!
  See merge request !9
* Add configuration for Tiago Iron
* octomap & track ik solver for MoveIt!
* Contributors: AleDF, Jordi Pages, davidfernandez

0.0.16 (2016-10-21)
-------------------
* fix maintainer
* add argument for steel and titanium versions
* add missing xml formatting
* add specific controllers for steel and titanium
* disable collision arm_5_link-gripper_link
* disable collision arm_6_link-wrist_ft_link
* add missing joints
* use soft links for steel and titanium srdf files
* disable collisions arm_5_link-gripper_link
* Contributors: Jordi Pages

0.0.15 (2016-07-08)
-------------------
* Merge branch 'add-titanium-collisions-with-ft' into 'cobalt-devel'
  add missing potential collisions with ft sensor frames
  See merge request !5
* add collisions with ft sensor
* Merge branch 'tiago_configs' into 'cobalt-devel'
  Added the 4 possible configurations of tiago_moveit_config
  See merge request !4
* Added the 4 possible configurations of tiago_moveit_config
* Contributors: Jordi Pages, Sam Pfeiffer, Victor Lopez

0.0.14 (2016-06-13)
-------------------
* Added necessary dependence to run moveit with a simulated or real robot
* Add disable collisions for force torque sensor
* Contributors: Sam Pfeiffer

0.0.13 (2016-06-01)
-------------------
* Added controllers for hand and gripper
* Contributors: Sam Pfeiffer

0.0.12 (2016-04-04)
-------------------
* Increase max speed of torso
* Contributors: Sam Pfeiffer

0.0.11 (2016-04-04)
-------------------
* Missing hand_palm_link in collision disables
* Contributors: Sam Pfeiffer

0.0.10 (2016-04-04)
-------------------
* Add disables in between hand finger links
  Without this, the robot will refuse to plan with closed hand
* Contributors: Sam Pfeiffer

0.0.9 (2016-03-31)
------------------
* Add disable collisions
  Using the generator.
  From:
  1300 / 2145 pairs disabled in tiago_titanium (845 enabled)
  To:
  2268 / 3096 pairs disabled in tiago_titanium (828 enabled)
* Add disable collisions
  Generated using https://gist.github.com/awesomebytes/18fe75b808c4c644bd3d a script that runs the urdf tree for adjacent links and checks for links without collision mesh to also disable the collision computation between them.
  From:
  (Generating matrix with max sampling density)
  329 / 465 pairs disabled in tiago_steel (136 enabled)
  To:
  754 / 873 pairs disabled in tiago_steel (119 enabled)
* Contributors: Sam Pfeiffer

0.0.8 (2016-03-18)
------------------
* Added impossible collision disabling between torso_fixed_column_link and arm_2_link
* Contributors: Sam Pfeiffer

0.0.7 (2016-03-18)
------------------
* Passing change to titanium too about torso_fixed_column_link collision with arm1 disabling
* Added another currently happening collision exception between torso_fixed_column_link and arm_1_link
* Contributors: Sam Pfeiffer

0.0.6 (2016-03-18)
------------------
* Add hand passive joints as passive
* added clear octomap and removed exceptions on collisions of arm wit hhead
* Contributors: Sam Pfeiffer

0.0.5 (2016-03-10)
------------------
* Refs #11489. Discard collisions between torsolinks
* Fix collisions with column
* Remove elements of prototype mobilebase
* Disable collision hand safety box <-> wrist mesh
* Add arm group + disable more internal hand collisions
* Contributors: Bence Magyar, jordi.pages@pal-robotics.com

0.0.4 (2015-05-20)
------------------
* Add hand_safety_box to the game!
* Disable more collisions between hand links
* Contributors: Bence Magyar

0.0.3 (2015-04-14)
------------------
* Fix gripper parts
* Add torso controller
* Separate configuration files for titanium and steel, launch files parametrized
* Contributors: Bence Magyar

0.0.2 (2015-01-20)
------------------
* Remove tiago_description dependency
* Contributors: Bence Magyar

0.0.1 (2015-01-20)
------------------
* Added configuration with arm controllers
* Initial version of tiago_moveit_config (no hand)
* Contributors: Sammy Pfeiffer
