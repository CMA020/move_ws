#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import zmq
#from msgpack import loads
import time
#import pyttsx
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from datetime import datetime
#from espeak import espeak
from moveo_moveit.msg import ArmJointState
from sensor_msgs.msg import JointState
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
cur_pos=[0,0,0,0,0]
pos=[0,0,0,0,0]
def joint_state_callback():
    

      
  """
  Callback function to process received joint state list_joint_values.
  
  Args:
      list_joint_values: The received JointState message.
  """
  # Access and process joint positions as needed. For example,
  # print current joint angles:
  while True:
      list_joint_values = _group.get_current_joint_values()
      print(list_joint_values)
      # current_state = _robot.get_current_state()
      # print(current_state)

      goal = ArmJointState()
      goal.position1 = int((1000 / 0.384) * (list_joint_values[4] - cur_pos[4]))
      goal.position2 = int((4000 / 0.63) * (list_joint_values[2] - cur_pos[2])) ##multiply by 10 in ard
      goal.position3 = int((4000 / 0.23) * (list_joint_values[1] - cur_pos[1]))
      goal.position4 = int((3000 / 0.58) * (list_joint_values[0] - cur_pos[0]))
      goal.position5 = int((6000 / 1.57) * (list_joint_values[3] - cur_pos[3]))
      goal.position6 = 0
      pub.publish(goal)
      rospy.sleep(13)


# publish detected object to a ros topic

if __name__ == '__main__':
    try:
        joint_state_callback()

    except rospy.ROSInterruptException:
        pass