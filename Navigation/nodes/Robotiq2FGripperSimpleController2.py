#!/usr/bin/env python

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep


def publisher():
    hoi=True
    rospy.init_node('Robotiq2FGripperSimpleController2')
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,queue_size=10)
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    pub.publish(command)
    rospy.sleep(3.0)
    while not rospy.is_shutdown():
		if(hoi==True):
			command.rPR=0
			pub.publish(command)
			rospy.sleep(2.0)
			hoi=False
			
		if(hoi==False):
			command.rPR=255
			pub.publish(command)
			rospy.sleep(2.0)
			hoi=True
							

if __name__ == '__main__':
    publisher()
