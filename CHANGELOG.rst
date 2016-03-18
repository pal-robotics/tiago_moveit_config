^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
