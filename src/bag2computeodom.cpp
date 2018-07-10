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

ros::Time current,current_imu;
ros::Time prev,prev_imu;

double x = 0;
double y = 0;
double theta = 0;

// Initial velocities
double x_dot = 0;
double y_dot = 0;
double thetadot = 0;

// Delta time
double dt,imu_dt;
int message_counter = 0;
int message_counter_imu = 0;
// Variable definitions
float v_rf_x,v_rf_y,w_rf,w_rf_imu,i_w_rf_imu;
double x_dot_avg,y_dot_avg,thetadot_avg;

// Variables for averaage filter
std::queue<double> xdot_queue;
std::queue<double> ydot_queue;
std::queue<double> thetadot_queue;
int Avg_Window = 25; 
int Avg_counter = 0;
double Kp_rot = 0.5;
double Kp_pos = 0.5;




double queueAvg(std::queue<double> myqueue)
{
	int size = myqueue.size();
	double avg = 0;
	int weight_sum = 1;
	// Weighted average (Last coming data more important)
	for (int i = 0; i < size; ++i)	
	{	
		avg += myqueue.front();
		weight_sum += (i);
		myqueue.pop();
	}
	return avg/weight_sum;

}

void encCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{


	//printf("dt:%f,p:%f,c:%f message_counter:%d \n", (current.toSec()-prev.toSec()),current.toSec(),prev.toSec(),message_counter);
	// To eliminate the encoder glitch
	if(dt > 0.005)
	{
		
		// Robot frame velocities are read
		v_rf_x =  msg->twist.linear.x; 
		v_rf_y =  msg->twist.linear.y; 
		w_rf = msg->twist.angular.z;
		// Robot frame velocities are converted to odometry frame
	}

}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "viz_codom");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/vel", 1000, encCallback);

	static tf::TransformBroadcaster br;
	ros::Rate r(1000);

	while(ros::ok())
	{
		ros::spinOnce();
		//printf("Hey2\n");
		current = ros::Time::now();
		message_counter++;
		if(message_counter == 1)
			prev = current;
		dt = (current-prev).toSec();
		if(dt > 0.005)
		{
			// Robot frame velocities are converted to odometry frame
			x_dot = v_rf_x * cos(theta) - v_rf_y * sin(theta);
			y_dot = v_rf_x * sin(theta) + v_rf_y * cos(theta);
			thetadot = w_rf;
			//thetadot = w_rf_imu;
			// Queue calculated value

			xdot_queue.push(x_dot);
			ydot_queue.push(y_dot);
			thetadot_queue.push(thetadot);

			
			// If more value than the window size, pop!, Otherwise cumulate

			if(xdot_queue.size() > Avg_Window )
			{
				xdot_queue.pop();
				ydot_queue.pop();
				thetadot_queue.pop();
			}

			// Take average of the windowed speeds
			x_dot_avg = queueAvg(xdot_queue);
			y_dot_avg = queueAvg(ydot_queue);
			thetadot_avg = queueAvg(thetadot_queue);



			// Position updates
			x += Kp_pos*x_dot_avg*dt;
			y += Kp_pos*y_dot_avg*dt;
			theta += Kp_rot*thetadot_avg*dt; 

			// Keep the previous timestamp for accurate calculation
			

			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);

			// Stamped pose is created and published here; 
			geometry_msgs::PoseStamped enc_pose;
			enc_pose.header.stamp = current;
			enc_pose.header.frame_id = "map";

			enc_pose.pose.position.x = x;
			enc_pose.pose.position.y = y;
			enc_pose.pose.position.z = 0;
			enc_pose.pose.orientation = q;

			// Stamped transformation is created and published
			geometry_msgs::TransformStamped enc_odom_trans;
			enc_odom_trans.header.stamp = current;
			enc_odom_trans.header.frame_id = "initial_pos";
			enc_odom_trans.child_frame_id = "computed_odom";

			enc_odom_trans.transform.translation.x = x;
			enc_odom_trans.transform.translation.y = y;
			enc_odom_trans.transform.translation.z = 0.0;
			enc_odom_trans.transform.rotation = q;

			br.sendTransform(enc_odom_trans);
		}
		prev = current;

		r.sleep();
	}
	return 0;
}

