#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math





rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
_planning_group = "arm"
s_commander = moveit_commander.roscpp_initialize(sys.argv)
_robot = moveit_commander.RobotCommander()
_scene = moveit_commander.PlanningSceneInterface()
_group = moveit_commander.MoveGroupCommander(_planning_group)
if __name__ == '__main__':
    while True :
        list_joint_values = _group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
