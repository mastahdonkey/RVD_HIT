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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)


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
joint_goal[5] = -pi/4
print("Homing!")

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# Calling stop() ensures that there is no residual movement
group.stop()

"""pose goal needs to be looked into later, but not necessary
#pose_goal = geometry_msgs.msg.Pose()
#pose_goal.orientation.w = 1.0
#pose_goal.position.x = 0.4
#pose_goal.position.y = 0.1
#pose_goal.position.z = 0.4
#group.set_pose_target(pose_goal)
#pose = group.go(wait=True)
"""

print("Press enter to make a square!")
raw_input()
waypoint = []
wpose = group.get_current_pose().pose
print(wpose.position)
wpose.position.z = 0.2
waypoint.append(copy.deepcopy(wpose))
wpose.position.x = -0.8
waypoint.append(copy.deepcopy(wpose))
wpose.position.y = -0.1
waypoint.append(copy.deepcopy(wpose))
wpose.position.x = -0.5
waypoint.append(copy.deepcopy(wpose))
wpose.position.y = 0.2
waypoint.append(copy.deepcopy(wpose))

print("Planning tajectory...")
(plan, fraction) = group.compute_cartesian_path(
                                   waypoint,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
print("Executing plan!")
group.execute(plan, wait=True)

# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()


















