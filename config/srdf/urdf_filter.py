#!/usr/bin/env python
from __future__ import print_function

from xml.dom.minidom import parse, Document
import sys

def get_direct_elements(parent, tag):

    for e in parent.getElementsByTagName(tag):
        if e.parentNode is parent:
            yield e

path = sys.argv[1]
start_link = sys.argv[2]
stop_links = set(sys.argv[3:])

tree = parse(path)
robot = tree.documentElement

links = dict((e.getAttribute("name"), e)  for e in get_direct_elements(robot, "link"))
joints = dict()
for joint in get_direct_elements(robot, "joint"):
    parent = next(get_direct_elements(joint, "parent")).getAttribute("link")
    child = next(get_direct_elements(joint,"child")).getAttribute("link")
    joints.setdefault(parent, dict())[child]=joint

walk = [ start_link ]
print("Start get idirect elements")
new_robot = robot.cloneNode(False)
while walk:
    link = walk.pop(0)
    new_robot.appendChild(links[link].cloneNode(True))
    for child, joint in joints.setdefault(link, dict()).items():
        if child not in stop_links:
            walk.append(child)
            new_robot.appendChild(joint.cloneNode(True))

new_robot.setAttribute("xmlns:xacro", "http://www.ros.org/wiki/xacro")
new_tree = Document()
new_tree.appendChild(new_robot)
print(new_tree.toxml())
