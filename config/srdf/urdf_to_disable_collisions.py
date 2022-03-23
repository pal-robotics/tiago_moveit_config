#!/usr/bin/env python
"""
Given a URDF given by either
 1) param server /robot_description
 2) as an argument

Generate all the MoveIt! SRDF
disable collision lines between adjacent
links and links without collision mesh.

Author: Sam Pfeiffer
"""
from __future__ import print_function
import sys
from urdf_parser_py.urdf import URDF


def traverse_from(from_link, child_map, level=0):
    """Traverse tree from link 'from_link' until end
    forming the disable collision lines"""
    next_childs = child_map[from_link]
    dcl = ""
    for joint, link in next_childs:
        # print "  " * level + str(link)
        if link in child_map:
            dcl += dcl_between(from_link, link)
            dcl += traverse_from(link, child_map, level=level + 1)
            # print "From " + from_link + " to " + link
        else:
            # print "No more childs..."
            # print "From " + from_link + " to " + link
            dcl += dcl_between(from_link, link)
    return dcl


def traverse_from_for_tree(from_link, child_map, link_map, level=0):
    """Traverse tree from link 'from_link' until end
     forming the ascii tree of links"""
    next_childs = child_map[from_link]
    tree = ""
    if link_map[from_link].collision is None:
        tree += "  " * level + str(from_link) + " (NO COLLISION)\n"
    else:
        tree += "  " * level + str(from_link) + "\n"
    for joint, link in next_childs:
        # tree += "  " * level + str(link) + "\n"
        if link in child_map:
            tree += traverse_from_for_tree(link,
                                           child_map,
                                           link_map,
                                           level=level + 1)
        else:
            if link_map[link].collision is None:
                tree += "  " * level + str(link) + " (NO COLLISION)\n"
            else:
                tree += "  " * level + str(link) + "\n"
    return tree


def dcl_between(link1, link2, reason="Adjacent"):
    link1, link2 = sorted((str(link1), str(link2)))
    """Create the disable collision line string between link1 and link2"""
    dcl = '  <disable_collisions link1="' + link1 + \
          '" link2="' + link2 + '" reason="' + reason + '" />\n'
    return dcl


def get_links_without_collision(links):
    """Given the list of links (robot.links) return a list
    with the links that dont have a collision mesh"""
    no_collision_links = []
    for l in links:
        if l.collision is None:
            no_collision_links.append(l.name)
    return no_collision_links


def get_links_with_collision(links):
    collision_links = []
    for l in links:
        if l.collision is not None:
            collision_links.append(l.name)
    return collision_links


def disable_all_vs_all(link_names):
    """Generate all combinations of links"""
    START = '  <disable_collisions link1="'
    MIDDLE = '" link2="'
    END = '" reason="Never" />\n'
    # print "We got links: " + str(link_names)
    already_disabled = []
    disable_lines = ""
    for idx, link1 in enumerate(sorted(link_names)):
        # print "  Link: " + str(link1) + " won't collide with: " +
        # str(link_names[idx + 1:])
        if len(already_disabled) > 0:
            # print "  Neither with already disabled: " + str(already_disabled)
            pass
        # print
        for link2 in link_names[idx:]:
            if link1 != link2:
                disable_lines += START + link1 + MIDDLE + link2 + END
        already_disabled.append(link1)

    return disable_lines


def disable_links_vs_no_col_links(col_links, no_col_links):
    """Generate disables between list of links that can actually collide
    with all the links that cant collide"""
    disabled = ""
    for link1 in col_links:
        for link2 in no_col_links:
            disabled += dcl_between(link1, link2, reason="Never")
    return disabled


if __name__ == '__main__':
    u = URDF()
    if len(sys.argv) == 1:
        robot = u.from_parameter_server()
    elif len(sys.argv) == 2:
        robot = u.from_xml_file(sys.argv[1])

    # Run URDF tree
    # print "\nTraversing all childs:"
    # print robot.child_map
    # print "From root:"
    # print robot.get_root()
    root = robot.get_root()

    # print "Tree looks like:"
    ascii_tree = traverse_from_for_tree(root, robot.child_map, robot.link_map)
    # print ascii_tree

    dcls = traverse_from(root, robot.child_map)

    # print "Generated disable collisions between adjacent links:"
    # print "=============="
    # print dcls
    adjacent_disabled = dcls

    # print "=============="
    # print "Now we generate the disable between all the links that have no collision mesh"
    # print "This is most probably not needed, but works as a URDF programming
    # exercise"

    # print "There are " + str(len(robot.links)) + " links"
    no_col = get_links_without_collision(robot.links)
    # print "From those, " + str(len(no_col)) + " links have no collision mesh"
    # print no_col
    # print "So we generate the exceptions of all links vs those links so they
    # aren't computed"
    no_collision_disabled = disable_all_vs_all(no_col)
    # print "These exceptions are:"
    # print no_collision_disabled

    # print "=============="
    # print "Now we generate the disabled between the rest of the links and"
    # print "the links without collision mesh"
    yes_col = get_links_with_collision(robot.links)
    collision_vs_no_collision_disableds = disable_links_vs_no_col_links(
        yes_col, no_col)
    # print "The list looks like:"
    # print collision_vs_no_collision_disableds

    # Now we should run the moveit setup assistant with a big number of
    # iterations to get the extra list of disabled collisions
    #
    # Then, we should merge this ones and those (which is just adding them)
    # Then, we should check that links that CAN ACTUALLY COLLIDE
    # aren't disabled (happened with arm and head...)
    # Which I don't really know how to do right now...
    # Fake "can_collide" tag to check if somewhere
    # that 2 links are disabled? that's a lot of manual work

    print()
    final_list = "<!-- Disabled because they are adjacent -->\n"
    final_list += "<!-- Tree looks like:\n"
    final_list += ascii_tree
    final_list += "\n -->\n"
    final_list += adjacent_disabled
    final_list += "\n<!-- Disabled because they don't have collision mesh so they can't collide between themselves-->\n"
    final_list += no_collision_disabled
    final_list += "\n<!-- Disables because the second links doesn't have collision mesh -->\n"
    final_list += collision_vs_no_collision_disableds

    # print "The final list looks like:"
    print(final_list)

    # with open('/tmp/collision_list.srdf', 'w') as f:
    #    f.write(final_list)
    # print "Written to /tmp/collision_list.srdf"


# Child map looks like:
# {'arm_1_link': [('arm_2_joint', 'arm_2_link')],
#  'arm_2_link': [('arm_3_joint', 'arm_3_link')],
#  'arm_3_link': [('arm_4_joint', 'arm_4_link')],
#  'arm_4_link': [('arm_5_joint', 'arm_5_link')],
#  'arm_5_link': [('arm_6_joint', 'arm_6_link')],
#  'arm_6_link': [('arm_7_joint', 'arm_7_link')],
#  'arm_7_link': [('arm_tool_joint', 'arm_tool_link')],
#  'arm_tool_link': [('gripper_joint', 'gripper_link')],
#  'base_footprint': [('base_footprint_joint', 'base_link'),
#   ('cover_joint', 'base_cover_link')],
#  'base_link': [('base_antenna_left_joint', 'base_antenna_left_link'),
#   ('base_antenna_right_joint', 'base_antenna_right_link'),
#   ('base_laser_joint', 'base_laser_link'),
#   ('base_mic_front_left_joint', 'base_mic_front_left_link'),
#   ('base_mic_front_right_joint', 'base_mic_front_right_link'),
#   ('base_mic_back_left_joint', 'base_mic_back_left_link'),
#   ('base_mic_back_right_joint', 'base_mic_back_right_link'),
#   ('base_imu_joint', 'base_imu_link'),
#   ('base_sonar_1_joint', 'base_sonar_1_link'),
#   ('base_sonar_2_joint', 'base_sonar_2_link'),
#   ('base_sonar_3_joint', 'base_sonar_3_link'),
#   ('wheel_right_joint', 'wheel_right_link'),
#   ('wheel_left_joint', 'wheel_left_link'),
#   ('caster_front_right_1_joint', 'caster_front_right_1_link'),
#   ('caster_front_left_1_joint', 'caster_front_left_1_link'),
#   ('caster_back_right_1_joint', 'caster_back_right_1_link'),
#   ('caster_back_left_1_joint', 'caster_back_left_1_link'),
#   ('torso_fixed_joint', 'torso_fixed_link'),
#   ('torso_fixed_column_joint', 'torso_fixed_column_link')],
#  'caster_back_left_1_link': [('caster_back_left_2_joint',
#    'caster_back_left_2_link')],
#  'caster_back_right_1_link': [('caster_back_right_2_joint',
#    'caster_back_right_2_link')],
#  'caster_front_left_1_link': [('caster_front_left_2_joint',
#    'caster_front_left_2_link')],
#  'caster_front_right_1_link': [('caster_front_right_2_joint',
#    'caster_front_right_2_link')],
#  'gripper_link': [('gripper_right_finger_joint', 'gripper_right_finger_link'),
#   ('gripper_left_finger_joint', 'gripper_left_finger_link'),
#   ('gripper_grasping_frame_joint', 'gripper_grasping_frame')],
#  'head_1_link': [('head_2_joint', 'head_2_link')],
#  'head_2_link': [('xtion_joint', 'xtion_link')],
#  'torso_fixed_link': [('torso_lift_joint', 'torso_lift_link')],
#  'torso_lift_link': [('head_1_joint', 'head_1_link'),
#   ('arm_1_joint', 'arm_1_link')],
#  'xtion_depth_frame': [('xtion_depth_optical_joint',
#    'xtion_depth_optical_frame')],
#  'xtion_link': [('xtion_optical_joint', 'xtion_optical_frame'),
#   ('xtion_depth_joint', 'xtion_depth_frame'),
#   ('xtion_rgb_joint', 'xtion_rgb_frame')],
#  'xtion_rgb_frame': [('xtion_rgb_optical_joint', 'xtion_rgb_optical_frame')]}
