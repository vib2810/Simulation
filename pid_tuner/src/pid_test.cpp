#include <ros/ros.h>
#include<iostream>
#include "prius_msgs/Control.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <time.h> 
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int16.h>


using namespace std;

//square wave
double kp=0.63359,ki=0.00015725,kd=26.3199;
//not square wave
//double kp=0.52732,ki=0.00010767,kd=26.4035;
//sino wave
// double kp=2.236,ki=0.0018662,kd=49.3545;
double yp = 0.8  ,yi = 0.05 ,yd = 0.5 ,y = 0.8;
ros::Publisher pub ;
ros::Publisher pub2 ;
// ros::Time start_time,currenttime,pasttime;
double error, desired=0, current, value , errorprior, error_diff=0, error_sum=0;
double filtered_error, filtered_error_diff, prev_filtered_error;
float countf =00;

// #A function to give a trapezoidal profile to desired velocity
// # ____________________________________________PEAK
// # ................/|         |\................
// # .............../ |         | \ ..............                   
// # ............../  |         |  \..............
// # ............./   |         |   \.............
// # ............/    |         |    \............
// # .........../     |         |     \...........
// # <---td--->/<-tr->|<---t--->|<-tr->\..........

// You can specify the profile in trap function using the tr, t, td, and peak values

void trap()
{
	float tr=800.0;
    float t=600.0;
    float td=500.0;
    float peak=35.0/3.6;
    float m=peak/(tr);

    countf=countf+1;
    if(countf>td && countf < (tr+td)) desired=m*countf - m*td;
    else if(countf>tr+t+td) desired = -m*countf+ (peak+m*(tr+t+td));
    if(countf >td+2*tr+t) countf=0; 
    if(desired<0) desired=0;
 }
void des(const  std_msgs::Int16::ConstPtr& data)
{
	desired=data->data;
}
float get_throttle(float vel) //function updated and will now use pid to give throttle
{
	current=vel; 

	error=(desired)-current;
	error_diff=(error-errorprior) ;
	error_sum=error_sum+(error);
	errorprior=error;

	value= (kp*error + ki*error_sum + kd*error_diff);
	if(value<=-1) value=-1;
	if(value>=1) value=1;
	return value;
}
float get_throttle_adap(float vel) //function updated and will now use pid to give throttle
{
	current=vel; 

	error=(desired)-current;
	error_diff=(error-errorprior) ;
	error_sum=error_sum+(error);
	errorprior=error;
	
	filtered_error = error * (1 - y) + errorprior* y ;
	filtered_error_diff = filtered_error - prev_filtered_error;
	prev_filtered_error = filtered_error;
	
	kp = kp + yp * fabs(error - filtered_error);
	ki = ki + yi * filtered_error;
	kd = kd + yd * fabs(error_diff - filtered_error_diff);
	if(kp > 2.5)	kp = 2.5;
	if(kp < 0.3)	kp = 0.3;
	if(ki > 0.010)      ki = 0.010;
	if(ki < 0.00001)	ki = 0.00001;
	if(kd > 30) kd = 30;
	if(kd < 10)	kd = 10;

	value= (kp*error + ki*error_sum + kd*error_diff);
	if(value<=-1) value=-1;
	if(value>=1) value=1;
	return value;

}
float get_throttle_mit(float vel) //function updated and will now use pid to give throttle
{
	current=vel; 

	error=(desired)-current;
	error_diff=(error-errorprior) ;
	error_sum=error_sum+(error);
	errorprior=error;
	
	kp = kp + yp * error * error;
	ki = ki + yi * error * error_sum;
	kd = kd + yd * error * error_diff;
	if(kp > 2.5)	kp = 2.5;
	if(kp < 0.3)	kp = 0.3;
	if(ki > 0.010)      ki = 0.010;
	if(ki < 0.00001)	ki = 0.00001;
	if(kd > 30) kd = 30;
	if(kd < 10)	kd = 10;

	value= (kp*error + ki*error_sum + kd*error_diff);
	if(value<=-1) value=-1;
	if(value>=1) value=1;
	return value;
}

void callback_feedback(const nav_msgs::Odometry::ConstPtr& data)
{
	trap();
	cout<<"desired"<< desired*3.6<< endl;
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
	float vel = (data->twist.twist.linear.x * cos(yaw) + data->twist.twist.linear.y * sin(yaw));
	cout<< "vel of car"<< vel*3.6<< endl;
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
