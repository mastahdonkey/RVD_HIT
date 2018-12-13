#!/usr/bin/env python
# BEGIN ALL
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
#import geometry_msg.msg
from math import pi
from std_msgs.msg import String, Bool, Int32
from moveit_commander.conversions import pose_to_list	
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Pick_and_place', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,queue_size=10)


# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print ("============ Reference frame: %s") % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print ("============ End effector: %s") % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print ("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print ("============ Printing robot state")
print (robot.get_current_state())
print ("")

# We can get the joint values from the group and adjust some of the values:

print("Press enter to home the arm")
raw_input()

joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/2
joint_goal[2] = -pi*3/4
joint_goal[3] = -pi*3/4
joint_goal[4] = -pi/2
joint_goal[5] = -pi/2
print("Homing!")

group.go(joint_goal, wait=True)

command = outputMsg.Robotiq2FGripper_robot_output();

#Reset
command.rACT = 0
pub.publish(command)
rospy.sleep(0.1)

#Activate
command.rACT = 1
command.rGTO = 1
command.rSP  = 255
command.rFR  = 150
pub.publish(command)
rospy.sleep(0.1)

# Calling stop() ensures that there is no residual movement
group.stop()


print("Press enter to make a square!")
raw_input()
waypoint = []
wpose = group.get_current_pose().pose
print(wpose.position)
wpose.position.z = 0.05
waypoint.append(copy.deepcopy(wpose))

print("Planning tajectory...")
(plan, fraction) = group.compute_cartesian_path(
                                   waypoint,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
print("Executing plan!")
group.execute(plan, wait=True)

#Close gripper
command.rPR=80
pub.publish(command)
rospy.sleep(0.1)

# Calling `stop()` ensures that there is no residual movement
group.stop()

group.clear_pose_targets()

print("Press enter to make a square!")
#raw_input()

wpose.position.z = 0.3
waypoint.append(copy.deepcopy(wpose))
wpose.position.x = -0.8
waypoint.append(copy.deepcopy(wpose))
wpose.position.z = 0.05
waypoint.append(copy.deepcopy(wpose))

print("Planning tajectory...")
(plan, fraction) = group.compute_cartesian_path(
                                   waypoint,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
print("Executing plan!1")
group.execute(plan, wait=True)

#Close gripper
command.rPR=0
pub.publish(command)
rospy.sleep(0.1)

group.stop()

# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()



















