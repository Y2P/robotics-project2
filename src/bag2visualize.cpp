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

void pubCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	ros::Time current =  ros::Time::now();
	++message_counter;
	static tf::TransformBroadcaster br;


	if(message_counter == 1)
	{
		i_x = msg->x;
		i_y = msg->y;
		i_theta = msg->theta;
	}	
	geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(msg->theta);

	if(message_counter%2 == 0)
		return;
	//	printf("Time: %.3f X:%.2f,Y:%.2f,Theta:%.2f\n",msg.header.stamp.toSec() ,msg.pose.position.x ,msg.pose.position.x , i->theta-i_theta  );

	// This is the tf between map and inital position
	// geometry_msgs::TransformStamped map2ip;
	// map2ip.header.stamp = current;
	// map2ip.header.frame_id = "map";
	// map2ip.child_frame_id = "initial_pos";
	// map2ip.transform.translation.x =  i_x ;
	// map2ip.transform.translation.y =  i_y;
	// map2ip.transform.translation.z = 0;
	// map2ip.transform.rotation =  tf::createQuaternionMsgFromYaw(i_theta);
	// br.sendTransform(map2ip);


	// This is the tf between initial position and computed odometry
	geometry_msgs::TransformStamped ip2codom;
	ip2codom.header.stamp = current;
	ip2codom.header.frame_id = "map";
	ip2codom.child_frame_id = "real_odom";
	ip2codom.transform.translation.x =  msg->x + i_x ;
	ip2codom.transform.translation.y =  msg->y + i_y ;
	ip2codom.transform.translation.z = 0;
	ip2codom.transform.rotation = tf::createQuaternionMsgFromYaw(msg->theta + i_theta);
	br.sendTransform(ip2codom);


	// This is the tf between computed odometry and base link
	geometry_msgs::TransformStamped odom2baselink;
	odom2baselink.header.stamp = current;
	odom2baselink.header.frame_id = "real_odom";
	odom2baselink.child_frame_id = "base_link";
	odom2baselink.transform.translation.x = -0.17;
	odom2baselink.transform.translation.y =  0;
	odom2baselink.transform.translation.z = 0.034;
	odom2baselink.transform.rotation = tf::createQuaternionMsgFromYaw(0);
	br.sendTransform(odom2baselink);

	ros::Time computationTime = ros::Time::now();
	printf("%f\n",(computationTime-current).toSec());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "viz_real");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/robot_markerset/ground_pose", 1000, pubCallback);
	ros::spin();
	return 0;
}