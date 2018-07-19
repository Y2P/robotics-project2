#include <fstream>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <tf2_msgs/TFMessage.h>
#include <ros/console.h>


std::string input_string;
int main(int argc, char *argv[])
{
	// This node publishes the user input for odometry source if it is valid
	ros::init(argc, argv, "sub");
	ros::NodeHandle n;
	// Publisher for user input string
	ros::Publisher input_pub = n.advertise<std_msgs::String>("input_str", 50);
	std_msgs::String inp;
	while(ros::ok())
	{	
		// Read input
		std::cin>>input_string;
		// If it is not valid, notify
		if (!(input_string.compare("ENC") == 0) && !(input_string.compare("IMU") == 0))
		{
			ROS_INFO("INVALID SOURCE");
		}else{
			// If it is valid, publish the input
			inp.data = input_string;
			input_pub.publish(inp);
		}
		
	}

	return 0;
}