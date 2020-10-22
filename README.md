# TIAGo MoveIt! Config

This package contains the MoveIt! config files for all possible TIAGo configurations (end effectors, force torque sensors..).

To make maintenance easier, there are templates of the files that differ between configurations. To understand how this template works please read https://github.com/pal-robotics/tiago_robot/ readme.



## Running the setup assistant

If you need to modify the setup assistant, you'll need to:

1. Provide arguments to the xacro file to get the variant of TIAGo that you need. For MoveIt! the parameters may need to modify are `arm`, `end_effector` and `ft_sensor`.
2. Create (or softlnk) a SRDF file at `config/tiago.srdf` if you want to load existing moveit configuration. The SRDF files used are in `config/srdf`, and only changes in that directory will not reflect on the robot.

Make sure that after running to reflect the changes in the auto generated files.