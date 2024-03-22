#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_7', 'joint_8']
        joint_state.position = [0.2551948098842871, -0.46394454967308035, 0.17301522945142694, -0.3056768768503315, 0.09736137393810315, 0.0, 0.0]
        joint_state.velocity = []
        joint_state.effort = []
        joint_state.header.stamp = rospy.Time.now()
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass