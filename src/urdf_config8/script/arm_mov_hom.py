#! /usr/bin/env python3
import rospy
import sys
import copy

from std_msgs.msg import String
import sys
import zmq
# from msgpack import loads
import time
# import pyttsx
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import multiprocessing as mp

import cv2
from datetime import datetime
# from espeak import espeak
from moveo_moveit.msg import ArmJointState
from sensor_msgs.msg import JointState
import os
fixated_object_label = 'a'
gripper = {'open': 0, 'a': 80, 'b': 40}
upright = [0, 0, 0, 0, 0, 0]

# predefined movements for pick and place of an apple and banana
apple_pick = [0, 1750, 0, -1400, -3300, gripper['a']]
apple_move = [1750, -1753, 0, 1400, 3300, gripper['a']]
apple_place = [-1750, 1750, 0, 1400, -3300, gripper['open']]

banana_pick = [0, -2243, -24410, 14, -400, gripper['b']]
banana_move = [0, -1043, -17410, 14, -3300, gripper['b']]
banana_place = [4600, -2400, -20410, -91, -400, gripper['open']]

rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
pub = rospy.Publisher('joint_steps', ArmJointState, queue_size=4)

rate = rospy.Rate(.1)  # 20hz
_planning_group = "arm"
s_commander = moveit_commander.roscpp_initialize(sys.argv)
_robot = moveit_commander.RobotCommander()
_scene = moveit_commander.PlanningSceneInterface()
_group = moveit_commander.MoveGroupCommander(_planning_group)
cur_pos = [0, 0, 0, 0, 0]
pos = [0, 0, 0, 0, 0]
joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
rate = rospy.Rate(10)
file_path = os.path.expanduser("~/catkin_ws/src/urdf_config8/script/joint_values.txt")
flag=0

flag2=0


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
            goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        # return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def joint_state_callback(queue):
    a=[]
    flag = 0
    ur5 = Ur5Moveit()

    """
    Callback function to process received joint state list_joint_values.

    Args:
        list_joint_values: The received JointState message.
    """
    # Access and process joint positions as needed. For example,
    # print current joint angles:


    while True:
        flag2 = queue.get()
        if flag==0 :

            with open(file_path, 'r') as file:
                # Read the contents of the file
                contents = file.read()

                # Split the contents into lines
                lines = contents.splitlines()

                # Iterate over the lines
                for line in lines:
                    # Convert the line (string) to a list
                    joint_values = eval(line)

                    # Print the joint values

            print(joint_values)
            ur5.set_joint_angles(joint_values)
            goal = ArmJointState()
            goal.position1 = int((1000 / 0.384) * (joint_values[4] - cur_pos[4]))
            goal.position2 = int((4000 / 0.63) * (joint_values[2] - cur_pos[2]))  ##multiply by 10 in ard
            goal.position3 = int((4000 / 0.23) * (joint_values[1] - cur_pos[1]))
            goal.position4 = int((3000 / 0.58) * (joint_values[0] - cur_pos[0]))
            goal.position5 = int((6000 / 1.57) * (joint_values[3] - cur_pos[3]))
            goal.position6 = 0
            pub.publish(goal)
            rospy.sleep(2)

            flag=1
        #print(pos, "posb")
        list_joint_values = _group.get_current_joint_values()
        #
        print(list_joint_values)
        # current_state = _robot.get_current_state()
        # print(current_state)
        pos[0]=(list_joint_values[0] - pos[0])
        pos[1] = (list_joint_values[1] - pos[1])
        pos[2] = (list_joint_values[2] - pos[2])
        pos[3] = (list_joint_values[3] - pos[3])
        pos[4] = (list_joint_values[4] - pos[4])

        goal = ArmJointState()
        goal.position1 = int((1000 / 0.384) * (list_joint_values[4] - cur_pos[4]))
        goal.position2 = int((4000 / 0.63) * (list_joint_values[2] - cur_pos[2]))  ##multiply by 10 in ard
        goal.position3 = int((4000 / 0.23) * (list_joint_values[1] - cur_pos[1]))
        goal.position4 = int((3000 / 0.58) * (list_joint_values[0] - cur_pos[0]))
        goal.position5 = int((6000 / 1.57) * (list_joint_values[3] - cur_pos[3]))
        goal.position6 = 0
        with open(file_path , 'w') as file:
            print("Writing...")
            #print(pos ,"posa")
            file.write(str(list_joint_values))
            #file.write(str(pos))
        pub.publish(goal)
        rospy.sleep(13)
        if flag2==1:

            goal = ArmJointState()
            goal.position1 = int((1000 / 0.384) * (cur_pos[4]))
            goal.position2 = int((4000 / 0.63) * (cur_pos[2]))  ##multiply by 10 in ard
            goal.position3 = int((4000 / 0.23) * (cur_pos[1]))
            goal.position4 = int((3000 / 0.58) * (cur_pos[0]))
            goal.position5 = int((6000 / 1.57) * (cur_pos[3]))
            goal.position6 = 0

            pub.publish(goal)

            rospy.sleep(13)

            break


def exit(queue):
    flag3=0
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):

            flag3=1
            while flag3==1:
                queue.put(1)



        else:
            queue.put(0)

# publish detected object to a ros topic

if __name__ == '__main__':
    queue = mp.Queue()
    s = mp.Process(target=joint_state_callback, args=(queue,))
    m = mp.Process(target=exit, args=(queue,))





    try:
        s.start()
        m.start()




    except rospy.ROSInterruptException:
        pass