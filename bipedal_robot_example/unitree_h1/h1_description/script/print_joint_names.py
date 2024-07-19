#!/bin/python

import rospy

from sensor_msgs.msg import JointState


def joint_state_callback(msg:JointState):
    print(msg.name)
    exit(0)
    
    
def main():
    rospy.init_node('print_joint_names')
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.spin()


main()