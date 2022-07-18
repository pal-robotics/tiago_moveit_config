# TIAGo MoveIt! Config

This package contains the MoveIt! config files for all possible TIAGo configurations (end effectors, force torque sensors..).

To make maintenance easier, there are templates of the files that differ between configurations. To understand how this template works please read https://github.com/pal-robotics/tiago_robot/ readme.

## Updating or adding support for an end effector

Prequisites: pal_moveit_config_generator

There is one *.srdf.xacro file in config/srdf/end_effectors for each supported end effector.

These files have to provide 2 xacro elements:
* property `end_effector_name`: eg. `gripper` or `hand`, will be passed as a prefix for all groups and links of that end effector
*  macro `define_end_effector` (`arm`, `name`): to inject end effector-specific SRDF elements like `group`, `end_effector`, and `passive_joint`. Special care needs to be taken to rename all dependent names with `${name}` (prefix of the end effector, derived from `end_effector_name`) or `${arm}` (name of the corresponding arm, used for `end_effector`)

Afterwards the SRDF files need to be regenerated (takes ~10 minutes):
```bash
./config/srdf/update.sh
```

The changes have to be reviewed and added/committed carefully.

## Running the setup assistant

If you need to modify the setup assistant, you'll need to:

1. Provide arguments to the xacro file to get the variant of TIAGo that you need. For MoveIt! the parameters may need to modify are `arm`, `end_effector`, `base_type` and `ft_sensor`.
2. Create (or softlnk) a SRDF file at `config/tiago.srdf` if you want to load existing moveit configuration. The SRDF files used are in `config/srdf`, and only changes in that directory will not reflect on the robot.

Make sure that after running to reflect the changes in the auto generated files.
