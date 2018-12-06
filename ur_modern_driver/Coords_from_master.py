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
from sensor_msgs.msg import JointState


def Home(Now):
	global status_home
	print("Press enter to home the arm")
	print(Now.data)
	if Now.data == True:
		status_home = True
		joint_goal = group.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi/2
		joint_goal[2] = -pi*3/4
		joint_goal[3] = -pi*3/4
		joint_goal[4] = -pi/2
		joint_goal[5] = -pi/4
		print("Homing!")
		# The go command can be called with joint values, poses, or without any
		#parameters if you have already set the pose or joint target for the group
		group.go(joint_goal, wait=True)
		# Calling stop() ensures that there is no residual movement
		group.stop()
		group.clear_pose_targets()
		
def Get_position(pos):
	global status_home
	if status_home == True:
		x = pos.position[0]
		y = pos.position[1]		
		z = pos.position[2]
	
	
		print('\nMoving...')
	
		waypoint = []
	
		wpose = group.get_current_pose().pose
	
		print(wpose.position)
		wpose.position.x = x
		wpose.position.y = y
		wpose.position.z = z
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
	


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

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
status_home=False
if __name__ == '__main__':	
	try:
		rospy.Subscriber('Home', Bool, Home)
		rospy.Subscriber('coordinatesCamera', JointState, Get_position)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass



















