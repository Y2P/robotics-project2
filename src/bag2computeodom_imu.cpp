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

ros::Time current_enc,current_imu;
ros::Time prev_enc,prev_imu;

double x = 0;
double y = 0;
double theta = 0;

// Initial velocities
double x_dot = 0;
double y_dot = 0;
double thetadot = 0;

// Delta time
double dt_enc= 0,dt_imu = 0;
int message_counter = 0;
int message_counter_imu = 0;
// Variable definitions
float acc_y = 0,acc_z = 0;

float v_rf_x = 0,v_rf_y = 0,w_rf = 0,w_rf_imu = 0,imu_offset = 0;
double x_dot_avg = 0,y_dot_avg  = 0,thetadot_avg = 0 ;

// Variables for averaage filter
std::queue<double> xdot_queue;
std::queue<double> ydot_queue;
std::queue<double> thetadot_queue;
int Avg_Window = 25; 
int Avg_WindowIMU =  300;
int Avg_counter = 0;
double Kp_rot = 1;
double Kp_pos = 0.5;
geometry_msgs::PoseStamped enc_pose;

int encFlag = 0;
int ImuFlag = 0;

// Complementary Filter Parameters
double gain = 0.99999;
double complementary_theta = 0;
double unbiased_acc_y,unbiased_acc_z,g_min = 3.5;
double delta_theta = 0;

// Averaging filter
double queueAvg(std::queue<double> myqueue)
{
	int size = myqueue.size();
	double avg = 0;
	int weight_sum = 0;
	if (size == 0)
	{
		return 0 ;
	}
	else
	{

		for (int i = 0; i < size; ++i)  
		{   
			avg += myqueue.front()*(i+1);
			weight_sum += (i+1);
			myqueue.pop();
		}
		avg = avg/(weight_sum);
		return  avg;
	
	}
}

void encCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{


	//printf("dt:%f,p:%f,c:%f message_counter:%d \n", (current.toSec()-prev.toSec()),current.toSec(),prev.toSec(),message_counter);
	// To eliminate the encoder glitch
	if(dt_enc > 0.005)
	{
		
		// Robot frame velocities are read
		v_rf_x =  msg->twist.linear.x; 
		v_rf_y =  msg->twist.linear.y; 
		w_rf = msg->twist.angular.z;
		// Robot frame velocities are converted to odometry frame
	}
	encFlag = 1;


}
void IMUCallback( const sensor_msgs::Imu::ConstPtr& imu){
	//callback every time the robot's angular velocity is received
	if ( abs(x_dot) < 0.1 && abs(y_dot) < 0.1  )
	{
		imu_offset = (imu_offset*message_counter_imu + imu->angular_velocity.x)/(message_counter_imu+1);
		message_counter_imu++;
   
	}

	acc_y = imu->linear_acceleration.y ;//+ 1.568076; 
	acc_z = imu->linear_acceleration.z ;//+ 0.005081; 
	//this block is to filter out imu noise
	/*
	if(fabs((-imu->angular_velocity.x -0.345463961363)) < 0.03)
	{
	   w_rf_imu = 0;
	   acc_y = 0;
	   acc_z = 0;
	}
	else
	{
		w_rf_imu =1.0*(-imu->angular_velocity.x -0.345463961363);
	}
	*/
	w_rf_imu =1.0*(-imu->angular_velocity.x -0.345463961363);	
	ImuFlag = 1;
 //   printf("%f\n",-imu->angular_velocity.x-0.345463961363);
/*
	printf("theta:%f\n", theta);
	printf("abs vel:%f\n", fabs(-imu->angular_velocity.x -0.345463961363) );
	printf("thetadot_avg:%f\n", thetadot);
 */
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "viz_codom_imu");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/vel", 1000, encCallback);
	ros::Subscriber imu_sub = n.subscribe("/imu", 100, IMUCallback);
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu_pose", 50);

	enc_pose.header.frame_id = "initial_pos";

	ros::Rate r(10000);
	theta = 0;

	while(ros::ok())
	{
		ros::spinOnce();



			if(encFlag == 1)
			{
				current_enc = ros::Time::now();
				message_counter++;
				if(message_counter == 1)
				{   
					dt_enc = 0;
					prev_enc = current_enc;
				}
				dt_enc = (current_enc-prev_enc).toSec();
				prev_enc = current_enc;

				encFlag = 0;

				if(dt_enc > 0.005 && dt_enc < 0.1)
				{
		// Robot frame velocities are converted to odometry frame
					x_dot = v_rf_x * cos(theta) - v_rf_y * sin(theta);
					y_dot = v_rf_x * sin(theta) + v_rf_y * cos(theta);
					// Queue calculated value

					xdot_queue.push(x_dot);
					ydot_queue.push(y_dot);

					
					// If more value than the window size, pop!, Otherwise cumulate

					if(xdot_queue.size() > Avg_Window )
					{
						xdot_queue.pop();
						ydot_queue.pop();
					}

					// Take average of the windowed speeds
					x_dot_avg = queueAvg(xdot_queue);
					y_dot_avg = queueAvg(ydot_queue);


					// Position updates
					x += Kp_pos*x_dot_avg*dt_enc;
					y += Kp_pos*y_dot_avg*dt_enc;   

					enc_pose.header.stamp = current_enc;
				}
			}
			else if(ImuFlag == 1)
			{
				current_imu = ros::Time::now();
				message_counter_imu++;
				dt_imu = (current_imu-prev_imu).toSec();

				if(message_counter_imu == 1)
				{   
					dt_imu = 0;
					prev_imu = current_imu;
				}
				prev_imu = current_imu;

				if ( dt_imu > 0.005 && dt_imu < 0.1)
				{


					ImuFlag = 0;
					thetadot = w_rf_imu;
					/*
					thetadot_queue.push(thetadot);
					if (thetadot_queue.size() > Avg_WindowIMU) 
					{
						thetadot_queue.pop();

					}
					thetadot_avg = queueAvg(thetadot_queue);
					*/


					// Debiasing the data source from IMU
					unbiased_acc_y = acc_y + 1.568076;
					unbiased_acc_z = acc_z + 0.005081;

					// Selecting g values above the threshold.
					// It allows us to add the IMU accelometer effect when it is important enough.
					if((fabs(unbiased_acc_y) + fabs(unbiased_acc_z)) >= g_min )
					{
						// Complementary filter calculation based on gain values for each source.
						complementary_theta = atan2(acc_y,acc_z);
						delta_theta = (thetadot*dt_imu)*(gain) + (1-gain)*complementary_theta;	
						theta += Kp_rot*delta_theta; 

						printf("%f\n",(fabs(unbiased_acc_y) + fabs(unbiased_acc_z)));
					}
					else
						theta += Kp_rot*thetadot*dt_imu; 
					 


	


					enc_pose.header.stamp = current_imu;
				}


			}
					
/*
*/
			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);

			// Stamped pose is created and published here; 


			enc_pose.pose.position.x = x;
			enc_pose.pose.position.y = y;
			enc_pose.pose.position.z = 0;
			enc_pose.pose.orientation = q;

			pose_pub.publish(enc_pose);

		

			r.sleep();
	}
	return 0;
}


