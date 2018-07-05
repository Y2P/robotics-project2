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
#define foreach BOOST_FOREACH
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <tf2_msgs/TFMessage.h>

/*
void pubCallback(const geometry_msgs::PoseStamped m)
{
	std::cout<<"Real Position x,y,theta:"<<m.pose.position.x<<","<<m.pose.position.y<<","<<m.pose.orientation.w<<std::endl;

}
*/
double queueAvg(std::queue<double> myqueue)
{
	int size = myqueue.size();
	double avg = 0;
	int weight_sum = 0;
	// Weighted average (Last coming data more important)
	for (int i = 0; i < size; ++i)	
	{	
		avg += myqueue.front()*(i+1);
		weight_sum += (i+1);
		myqueue.pop();
	}
	return avg/weight_sum;

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv,"encoder_odometry_publisher");
	ros::NodeHandle n;
	ros::Publisher encoder_odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);	
	tf::TransformBroadcaster transform_broadcaster;
	ros::Publisher encoder_pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);	


//	ros::Subscriber sub = n.subscribe("/publish_poses", 1000, pubCallback);
//	ros::spin();
	// Initial positions
	double x,y,theta;
	x = 0;
	y = 0;
	theta = 0;

	// Initial velocities
	double x_dot,y_dot,thetadot;
	x_dot = 0;
	y_dot = 0;
	thetadot = 0;

	// Delta time
	double dt;
	int message_counter = 0;

	// Variable definitions
	float v_rf_x,v_rf_y,w_rf;
	double x_dot_avg,y_dot_avg,thetadot_avg;
	// For bag file
	rosbag::Bag bag;
	bag.open("first_bag.bag", rosbag::bagmode::Read);
	std::vector<std::string> topics;
	topics.push_back(std::string("/vel"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	// For recorded bag file 
	rosbag::Bag out_bag;
	out_bag.open("enc_pose_comparison.bag", rosbag::bagmode::Write);

	// Variables for averaage filter
	std::queue<double> xdot_queue;
	std::queue<double> ydot_queue;
	std::queue<double> thetadot_queue;
	int Avg_Window = 500; 
	int Avg_counter = 0;
	double dt_sum = 0;
	double Kp_rot = 0.55;
	double Kp_pos = 0.65;
	ros::Time current;
	ros::Time prev;
	//while(ros::ok())
	//{

	tf2_msgs::TFMessage tf_enc;
	tf_enc.transforms.resize(1);
	tf2_msgs::TFMessage tf_real;
	tf_real.transforms.resize(3);
		// Main Loop
		foreach(rosbag::MessageInstance const m, view)
		{
			message_counter++;
			// Data is read from message
			geometry_msgs::TwistStamped::ConstPtr i = m.instantiate<geometry_msgs::TwistStamped>();
			// Timestamp that the data is taken
			current = m.getTime();
			// For first message, ignore it.
			if(message_counter == 1)
				prev = current;
			dt = (current-prev).toSec();

			// To eliminate the encoder glitch
			if(dt > 0.005)
			{
				//std:: cout <<"Delta time:"<< dt<<" sec"<< "\r" << std::flush;
				
				// Robot frame velocities are read
				v_rf_x =  i->twist.linear.x; 
				v_rf_y =  i->twist.linear.y; 
				w_rf = i->twist.angular.z;
				// Robot frame velocities are converted to odometry frame
				x_dot = v_rf_x * cos(theta) - v_rf_y * sin(theta);
				y_dot = v_rf_x * sin(theta) + v_rf_y * cos(theta);
				thetadot = w_rf;

				// Queue calculated value
				xdot_queue.push(x_dot);
				ydot_queue.push(y_dot);
				thetadot_queue.push(thetadot);
				
				Avg_counter ++;
				// If more value than the window size, pop!, Otherwise cumulate

				if(xdot_queue.size() > Avg_Window )
				{
					xdot_queue.pop();
					ydot_queue.pop();
					thetadot_queue.pop();
				
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

					out_bag.write("enc_pose",current,enc_pose);
					
					geometry_msgs::TransformStamped enc_odom_trans;
					enc_odom_trans.header.stamp = current;
					enc_odom_trans.header.frame_id = "initial_pos";
					enc_odom_trans.child_frame_id = "computed_odom";

					enc_odom_trans.transform.translation.x = x;
					enc_odom_trans.transform.translation.y = y;
					enc_odom_trans.transform.translation.z = 0.0;
					enc_odom_trans.transform.rotation = q;
					/*
					// This part is inspired from here: 
					// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
					// We publish it as transform
					

					tf_enc.transforms[0] = enc_odom_trans;
					out_bag.write("enc_tf",current,tf_enc);
					*/
					//send the transform


				}
			}
			prev = current;

		}
		// At this part true poses are recorded to same bag file 
		// For comparison

		// For reading bag
		topics.pop_back();
		topics.push_back(std::string("/robot_markerset/ground_pose"));
		rosbag::View view2(bag, rosbag::TopicQuery(topics)); 


    message_counter = 0;
	double i_x,i_y,i_theta;
	//while(ros::ok())
	//{
		foreach(rosbag::MessageInstance const m, view2)
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
			

			geometry_msgs::PoseStamped enc_pose;
			enc_pose.header.stamp = current;
			enc_pose.header.frame_id = "map";

			enc_pose.pose.position.x = i->x-i_x-0.17;
			enc_pose.pose.position.y = i->y-i_y;
			enc_pose.pose.position.z = 0+0.034;
			enc_pose.pose.orientation = q;

			out_bag.write("real_pose",current,enc_pose);

			// This is the tf between map and inital position
			geometry_msgs::TransformStamped map2ip;
			map2ip.header.stamp = current;
			map2ip.header.frame_id = "map";
			map2ip.child_frame_id = "initial_pos";
			map2ip.transform.translation.x =  i_x ;
			map2ip.transform.translation.y =  i_y;
			map2ip.transform.translation.z = 0;
			map2ip.transform.rotation =  tf::createQuaternionMsgFromYaw(0);
			tf_real.transforms[0] = map2ip;


			// This is the tf between initial position and computed odometry
			geometry_msgs::TransformStamped ip2codom;
			ip2codom.header.stamp = current;
			ip2codom.header.frame_id = "initial_pos";
			ip2codom.child_frame_id = "computed_odom";
			ip2codom.transform.translation.x =  i->x ;
			ip2codom.transform.translation.y =  i->y;
			ip2codom.transform.translation.z = 0;
			ip2codom.transform.rotation = q;
			tf_real.transforms[1] = ip2codom;


			// This is the tf between computed odometry and base link
			geometry_msgs::TransformStamped codom2baselink;
			codom2baselink.header.stamp = current;
			codom2baselink.header.frame_id = "computed_odom";
			codom2baselink.child_frame_id = "base_link";
			codom2baselink.transform.translation.x = -0.17;
			codom2baselink.transform.translation.y =  0;
			codom2baselink.transform.translation.z = 0.034;
			codom2baselink.transform.rotation = tf::createQuaternionMsgFromYaw(0);
			tf_real.transforms[2] = codom2baselink;

			out_bag.write("tf",current,tf_real);


			//odom_broadcaster.sendTransform(codom2baselink);

		}
	//}
	out_bag.close();
	bag.close();
	return 0;
}