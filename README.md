# Robotics Project 2
# This project presents ROS executables that can collaboratively run together to solve;
1) Odometry Computation from different sensor sources. Namely, 
	-  Encoder 
	-  IMU + Encoder
2) Map generation with best computed odometry and advanced tool of ROS called gmapping(Implements particle filter, scan matching etc.)
3) Navigation of the robot for given map by using advanced tool of ROS called AMCL

## How To Run
1) From source files(.cpp), workspace and package creation and compilation of nodes is needed. 
	- To create workspace for ROS:[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
	- To create a package under a workspace:[Create a package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
	- After these steps, place the source file under the corresponding package and in the terminal, enter following at the workspace directory:
		- catkin_make
2) After creation of environment, open a terminal and run "roscore"
3) There are three particular launcher files under the launch folder. The first one, called "roslaunch real_vs_computed.launch", aims to visualize ground-truth and computed odometry as transformation frame in the same frame. To launch this file, use this command at the directory of launcher;
	- roslaunch real_vs_computed.launch
4) As aforementioned, there are two sources of computed odometry, to switch between them, "switch" and "input" nodes are implemented. For switching, while the previous launch file running, run the input node with following code in a seperate terminal;
	-rosrun <package_name> input
5) To visualize overall procedure, run rviz and add the corresponding configuration file(real_vs_computed_odom.rviz) under rviz folder. You can visually examine the error between computed and real odometry and switch between sources by using the terminal input node is running.
6) [MAPPING] For the mapping, use the second launcher file, called "map_generation.launch" at the launcher folder directory, with following command;
	-roslaunch map_generation.launch
7) You can visualize the generated map, drift in robot's odometry by using rviz. After opening of rviz, open corresponding rviz file(map_generation.rviz) from the rviz folder
	-rviz
8) [NAVIGATION] The last part of the project is navigation of robot for given map. For this purpose, it is again enough to run corresponding launcher from launcher directory(navigation.launch) and open corresponding rviz file(navigation.rviz). Also note that, during 6,7 and 8 th steps(mapping and navigation), it is still possible to choose the odometry source by using "input" node, with same procedure in 4th step. However, in mapping, switching source from ENC to IMU or vice versa, results in with a wrong map due to error between them. It is healtier that use only only one odometry source during map generation.