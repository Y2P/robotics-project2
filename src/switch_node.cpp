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

// Default Encoder Odometry is chosen
std::string switch_string = "ENC";
std::string prev_switch_string = "ENC";

void poseListenerENC(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// Read the incoming pose
	static tf::TransformBroadcaster br;
	geometry_msgs::TransformStamped ENC;

	// If chosen source is only encoder, put the pose to a transformation frame and publish it
	if (switch_string.compare("ENC") == 0)
	{
		ENC.header.stamp = msg->header.stamp;
		ENC.header.frame_id = "computed_odom";
		ENC.child_frame_id = "base_link_2";

		ENC.transform.translation.x = msg->pose.position.x;
		ENC.transform.translation.y = msg->pose.position.y;
		ENC.transform.translation.z = 0.0;
		ENC.transform.rotation = msg->pose.orientation;

		br.sendTransform(ENC);
		prev_switch_string = switch_string;

	}
}
void poseListenerIMU(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;		
	geometry_msgs::TransformStamped IMU;
	// If chosen sources are encoder and IMU, put the pose to a transformation frame and publish it
	if(switch_string.compare("IMU") == 0)
	{
		IMU.header.stamp = msg->header.stamp;
		IMU.header.frame_id = "computed_odom";
		IMU.child_frame_id = "base_link_2";

		IMU.transform.translation.x = msg->pose.position.x;
		IMU.transform.translation.y = msg->pose.position.y;
		IMU.transform.translation.z = 0.0;
		IMU.transform.rotation = msg->pose.orientation;

		br.sendTransform(IMU);
		prev_switch_string = switch_string;
	}

}


void inputListener(const std_msgs::String::ConstPtr& msg)
{
	// Listen incoming input
	switch_string = msg->data;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "sub");
	ros::NodeHandle n;
	ros::Subscriber ENCpose_Listener = n.subscribe("/enc_pose", 50, poseListenerENC);
	ros::Subscriber IMUpose_Listener = n.subscribe("/imu_pose", 50, poseListenerIMU);
	ros::Subscriber input_Listener = n.subscribe("/input_str", 50, inputListener);

	ros::Rate r(1000);

	while(ros::ok())
	{	
		ros::spinOnce();
		r.sleep();

	}

	return 0;
}