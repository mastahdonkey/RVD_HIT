# ros_hit_project

This project is started in september 2018 <br />
OS: Ubuntu 16.04 <br />
ROS: Kinectic 16.04 with ROS industrial and Move-It!<br />
<br />
In this project we use a UR10 via ethernet and a Robotiq gripper via modbus RTU.<br />
We used move-it as the path planner for the arm and the 2 finger RTU node of the Robotiq gripper to open and close it<br />
This package contains the ur_modern_driver with the hardware_interface ABI changes in kinetic (https://github.com/iron-ox/ur_modern_driver/commit/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c) fixed to make it work with kinetic<br />
<br />
For reasons unknows the Robotiq package doesn't push, below you can find how we constructed it.<br />
Robotiq<br />
<br />
Go back to root<br />
$ cd<br />
<br />
Go to the src of your workspace<br />
$ cd catkin_ws/src<br />
<br />
Collect the kinetic branch of robotiq<br />
$ git clone -b kinetic-devel https://github.com/ros-industrial/robotiq.git<br />
<br />
Go back to catkin_ws<br />
$ cd ..<br />
<br />
Fix dependencies<br />
$ rosdep install --from-paths /path/to/your/catkin_ws/src --ignore-src<br />
<br />
Make the packages<br />
$ catkin_make<br />
<br />
$ cd<br />
$ sudo usermod -a -G dialout YOURUSERNAME -> In my case, mitchel<br />
<br />
Relog the pc to apply the usermod<br />
<br />
Running the gripper<br />
<br />
$ roscore<br />
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0<br />
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py<br />
<br />
To run the gripper with the robot under the same core:<br />
We have made a new launch file called ur10_bringup_HiT, Its the same as ur_10_bringup, but this also starts the<br /> grippercontroller with device:="/dev/ttyUSB0" as default:<br />
<br/>
$ roslaunch ur_modern_driver ur10_bringup_HiT robot_ip:="IP_OF_THE_ROBOT"<br />
<br/>
Don't forget to use a static IP adress on your pc as well!<br />
<br /><br/>
Startup commands:<br />
Only the arm: <br/>
$ roslaunch ur_modern_driver ur10_bringup.launch limited:="true" robot_ip:="IP_OF_YOUR_ROBOT"<br />
<br/>
Arm plus gripper: <br/>
$ roslaunch ur_modern_driver ur10_bringup_HiT.launch limited:="true" robot_ip:="IP_OF_THE_ROBOT"<br />
<br/>
Only the gripper: <br/>
$ roscore<br />
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0<br />
<br/>
To start the pathplanner of the arm: <br/>
$ roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true<br/>
<br/>
It won't move like this, but you can start planning using move_group in a python script. The joint_group name is "manipulator" for the UR10<br/>
<br/>
To start the controller of the gripper:<br/>
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py<br/>
<br/>

