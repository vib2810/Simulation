#include <ros/ros.h>
#include<iostream>
#include "prius_msgs/Control.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <time.h> 
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int16.h>


using namespace std;

const double kp=2.7657,ki=0.002994,kd=26.6325;
ros::Publisher pub ;
ros::Publisher pub2 ;
// ros::Time start_time,currenttime,pasttime;
double error, desired=0, current, value , errorprior,d,i=0;
float countf =300;
void trap()
{
	countf=countf+0.5;
	if(countf>400 && countf < 800) desired=0.0375*countf -15;
	else if(countf>1200) desired = -0.0375*countf+60;
	if(countf >1600) countf=0; 
}
void des(const  std_msgs::Int16::ConstPtr& data)
{
	desired=data->data;
}
float get_throttle(float vel) //function updated and will now use pid to give throttle
{
	current=vel; //a value between 0 and 1023;
	error=desired-current;
	d=(error-errorprior) ;
	i=i+(error);
	value= (kp*error + ki*i + kd*d);
	errorprior=error;
	if(value<=-1) value=-1;
	if(value>=1) value=1;
	return value;
}
void callback_feedback(const nav_msgs::Odometry::ConstPtr& data)
{
	// if(countf%10!=0) return;
	// Assigns the position of the robot to global variables from odometry.
	// :param x_bot [float]
	// :param y_bot [float]
	// :param yaw [float]
	// :param vel [float]
	trap();
	cout<<"desired"<< desired<< endl;
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

	// cout<<"x of car:"<<x_bot<<endl;
	// cout<<"y of car:"<< y_bot<<endl;
	// cout<<"angle of car:"<<endl;
	// cout<<"c"<<endl;
	float vel = (data->twist.twist.linear.x * cos(yaw) + data->twist.twist.linear.y * sin(yaw));
	if(vel>1) cout<<countf<<endl;
	cout<< "vel of car"<< vel<< endl;
	float throttle= get_throttle(vel);
	cout<< "Car throttle: "<< throttle<<endl;
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
	info.z=desired;
	pub.publish(cmd);
	pub2.publish(info);

}
int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pid_test");
	ros::NodeHandle nh;
	pub = nh.advertise<prius_msgs::Control>("/prius", 10); //publisher for throttle
	pub2 = nh.advertise<geometry_msgs::Point32>("info", 10); //publisher for data capture

  	ros::Subscriber odom_sub = nh.subscribe("base_pose_ground_truth",10, callback_feedback); //subscriber for listening to feedback
  	ros::Subscriber des_sub = nh.subscribe("des",10, des); //subscriber for listening to feedback
  	ros::spin();
}
