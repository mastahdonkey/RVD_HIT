#!/usr/bin/env python
# BEGIN ALL
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
# import geometry_msg.msg
from math import pi
from std_msgs.msg import String, Bool, Int32
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState
import roslib;

roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
command = outputMsg.Robotiq2FGripper_robot_output();


def grijper(action):
    global pub
    global command

    if action == "reset":
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0
        pub.publish(command)
        rospy.sleep(0.1)

    if action == "activate":
        # Activate
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
        pub.publish(command)
        rospy.sleep(0.1)

    if action == "open":
        command.rPR = 0
        pub.publish(command)
        rospy.sleep(1)

    if action == "close":
        command.rPR = 80
        pub.publish(command)
        rospy.sleep(1)


def Home(Now):
    global status_home

    print("Press enter to home the arm")
    print(Now.data)
    if Now.data == True:
        status_home = True

        grijper("reset")

        group.set_max_velocity_scaling_factor(0.3)
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi / 2
        joint_goal[2] = -pi * 3 / 4
        joint_goal[3] = -pi * 3 / 4
        joint_goal[4] = -pi / 2
        joint_goal[5] = -pi / 2

        print("Homing!")
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)
        # Calling stop() ensures that there is no residual movement
        group.stop()

        group.clear_pose_targets()

        grijper("activate")


def Get_position(pos):
    global status_home

    if status_home == True:
        status_home = False
        xpick = pos.position[0]
        ypick = pos.position[1]
        zpick = pos.position[2]
        xplace = pos.position[3]
        yplace = pos.position[4]
        zplace = pos.position[5]
        velpick = pos.velocity[0]
        velplace = pos.velocity[1]

        waypoint = []
        wpose = group.get_current_pose().pose
# ____________________________________________________________________________________________________________________ #
        print("go to standard pickup position")
        wpose.position.x = xpick
        wpose.position.y = ypick
        waypoint.append(copy.deepcopy(wpose))
        wpose.position.z = zpick  # 0.04
        waypoint.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
            waypoint,  # waypoints to follow
            0.02,  # eef_step
            0.0)  # jump_threshold
        group.execute(plan, wait=True)
        group.stop()
        waypoint = []
        group.clear_pose_targets()

# ____________________________________________________________________________________________________________________ #
        print("Closing gripper")
        grijper("close")

# ____________________________________________________________________________________________________________________ #
        print("go a bit up")
        wpose = group.get_current_pose().pose
        wpose.position.z = zpick + 0.2  # 0.2
        waypoint.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
            waypoint,  # waypoints to follow
            0.02,  # eef_step
            0.0)  # jump_threshold
        group.execute(plan, wait=True)
        group.stop()
        waypoint = []
        group.clear_pose_targets()

        # ____________________________________________________________________________________ #

        print("go to the place position")

        wpose = group.get_current_pose().pose
        wpose.position.x = xplace
        wpose.position.y = yplace
        waypoint.append(copy.deepcopy(wpose))
        wpose.position.z = zplace
        waypoint.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
            waypoint,  # waypoints to follow
            0.02,  # eef_step
            0.0)  # jump_threshold
        group.execute(plan, wait=True)
        group.stop()
        waypoint = []
        group.clear_pose_targets()

        # ____________________________________________________________________________________ #
        print("Opening gripper")
        grijper("open")
        wpose = group.get_current_pose().pose
        wpose.position.z = wpose.position.z + 0.2
        waypoint.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
            waypoint,  # waypoints to follow
            0.02,  # eef_step
            0.0)  # jump_threshold
        group.execute(plan, wait=True)
        group.stop()
        waypoint = []
        group.clear_pose_targets()


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Coords_from_master', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Reference frame: %s") % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("============ End effector: %s") % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# We can get the joint values from the group and adjust some of the values:
status_home = False
if __name__ == '__main__':

    try:

        rospy.Subscriber('Home', Bool, Home)
        rospy.Subscriber('coordinates', JointState, Get_position)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
