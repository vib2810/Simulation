#include <ros/ros.h>
#include<iostream>
#include "prius_msgs/Control.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <time.h> 
#include <geometry_msgs/Point32.h>


using namespace std;
ros::Publisher pub ;
ros::Publisher pub2 ;
clock_t start_time;

float get_throttle()
{
	clock_t ct=clock();
	float time=(float)(ct-start_time)/42000;
	float throttle=0;
	if(time>2) throttle=-1;
	if(time > 4) throttle=0;
	if(time>6 ) throttle=1;
	if(time>8) throttle=0;
	cout<< "throttle:" << throttle<< " time:"<< time<< endl;
	return throttle;
}
void callback_feedback(const nav_msgs::Odometry::ConstPtr& data)
{
	// Assigns the position of the robot to global variables from odometry.
	// :param x_bot [float]
	// :param y_bot [float]
	// :param yaw [float]
	// :param vel [float]
	prius_msgs::Control cmd;
	float x_bot = data->pose.pose.position.x;
	float y_bot = data->pose.pose.position.y;
	
	// quarternion to euler conversion
	float siny = 2.0 * (data->pose.pose.orientation.w *
				   data->pose.pose.orientation.z +
				   data->pose.pose.orientation.x *
				   data->pose.pose.orientation.y);
	float cosy = 1.0 - 2.0 * (data->pose.pose.orientation.y *
						 data->pose.pose.orientation.y +
						 data->pose.pose.orientation.z *
						 data->pose.pose.orientation.z);

	float yaw = atan2(siny, cosy) ;//yaw in radians

	cout<<"x of car:"<<x_bot<<endl;
	cout<<"y of car:"<< y_bot<<endl;
	cout<<"angle of car:"<<endl;
	cout<<"c"<<endl;

	float vel = (data->twist.twist.linear.x * cos(yaw) + data->twist.twist.linear.y * sin(yaw));
	cout<< "vel of car"<< vel<< endl;
	float throttle= get_throttle();
	if(throttle>=0)
	{
		cmd.shift_gears=cmd.FORWARD;
		cmd.throttle=throttle;
	}
	if(throttle<0)
	{
		cmd.shift_gears=cmd.REVERSE;
		cmd.throttle=-throttle;
	}
	geometry_msgs::Point32 info;
	info.x=throttle;
	info.y=vel;
	pub.publish(cmd);
	pub2.publish(info);

}
int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pid_tuner");
	ros::NodeHandle nh;
	pub = nh.advertise<prius_msgs::Control>("/prius", 10); //publisher for throttle
	pub2 = nh.advertise<geometry_msgs::Point32>("info", 10); //publisher for throttle

  	ros::Subscriber odom_sub = nh.subscribe("base_pose_ground_truth",10, callback_feedback); //subscriber for listening to feedback
  	start_time = clock(); 
  	ros::spin();
}
