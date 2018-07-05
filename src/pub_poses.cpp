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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
int main(int argc, char  *argv[])
{	
	ros::init(argc, argv, "pub");	

	static int i = 0;
	

	ros::NodeHandle n;
	ros::Publisher real_odom_pub = n.advertise<nav_msgs::Odometry>("real_odom", 1000);	
	tf::TransformBroadcaster odom_broadcaster;

	// For reading bag
 	rosbag::Bag bag;
    bag.open("first_bag2.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/robot_markerset/ground_pose"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Time current;
    geometry_msgs::PoseStamped msg;

	rosbag::Bag out_bag;
	out_bag.open("enc_pose_comparison.bag", rosbag::bagmode::Write);


    int message_counter = 0;
	double i_x,i_y,i_theta;
	//while(ros::ok())
	//{
		foreach(rosbag::MessageInstance const m, view)
		{	
			current =  m.getTime();
			++message_counter;

			geometry_msgs::Pose2D::ConstPtr i = m.instantiate<geometry_msgs::Pose2D>();

			if(message_counter == 1)
			{
				i_x = i->x;
				i_y = i->y;
				i_theta = i->theta;
			}	
			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(i->theta);

		//	printf("Time: %.3f X:%.2f,Y:%.2f,Theta:%.2f\n",msg.header.stamp.toSec() ,msg.pose.position.x ,msg.pose.position.x , i->theta-i_theta  );
			nav_msgs::Odometry real_odom;
			real_odom.header.stamp = m.getTime();
			real_odom.header.frame_id = "map";

			// After point published tf of odometry is created
			real_odom.pose.pose.position.x = i->x - i_x;
			real_odom.pose.pose.position.y = i->y - i_y;
			real_odom.pose.pose.position.z = 0;
			real_odom.pose.pose.orientation = q;


			geometry_msgs::PoseStamped enc_pose;
			enc_pose.header.stamp = current;
			enc_pose.header.frame_id = "map";

			enc_pose.pose.position.x = i->x-i_x;
			enc_pose.pose.position.y = i->y-i_y;
			enc_pose.pose.position.z = 0;
			enc_pose.pose.orientation = q;

			out_bag.write("real_pose",current,enc_pose);

			



			real_odom_pub.publish(real_odom);

			geometry_msgs::TransformStamped enc_odom_trans;
			enc_odom_trans.header.stamp = current;
			enc_odom_trans.header.frame_id = "map";
			enc_odom_trans.child_frame_id = "base_link";

			enc_odom_trans.transform.translation.x = i->x ;
			enc_odom_trans.transform.translation.y = i->y ;
			enc_odom_trans.transform.translation.z = 0;
			enc_odom_trans.transform.rotation = q;

			odom_broadcaster.sendTransform(enc_odom_trans);

		}
	//}

	//std_msgs::String msg;
	//std::stringstream ss;
	//ss << "HelloWorld";
	return 0;
}