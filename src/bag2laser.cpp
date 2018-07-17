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
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <tf2_msgs/TFMessage.h>

int message_counter = 0;
double i_x,i_y,i_theta;
sensor_msgs::LaserScan new_scan;

void pubCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	// This is the tf between computed odometry and base link
	new_scan = *msg;
	new_scan.header.frame_id = "laser_codom";



}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 1000, pubCallback);
	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("new_scan",50);
	
	ros::Rate r(1000);

	while(ros::ok())
	{
		ros::spinOnce();		
		pub.publish(new_scan);
		r.sleep();		
	}
	return 0;
}
