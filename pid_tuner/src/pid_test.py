#!/usr/bin/env python
# Node name       - pid_test
# Publishe topic  - cmd_vel (Twist), info(Point32)
# Subscribe topic - odometry/filtered1 (Odometry), 
# Author: Vibhakar Mohta
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
import thread
import time
global pub,pub1
throttle=0
d=0
i=0
errorprior=0
desired=0
kp=1
ki=1
kd=1
d=0
i=0
count=0
countf=0

#A function to give a trapezoidal profile to desired velocity
# ____________________________________________PEAK
# ................/|         |\................
# .............../ |         | \ ..............                   
# ............../  |         |  \..............
# ............./   |         |   \.............
# ............/    |         |    \............
# .........../     |         |     \...........
# <---td--->/<-tr->|<---t--->|<-tr->\..........

#You can specify the profile in trap function using the tr, t, td, and peak values

def trap():
    global countf, desired
    tr=500.0
    t=1000.0
    td=200.0
    peak=15.0
    m=peak/(tr-td)
    t2=((tr+t)*(-m)-peak)/(-m)

    countf=countf+1
    # print "tr=" +str(tr) + " t2=" +str(t2)  + " t="+str(countf)
    if(countf>td and countf < tr): 
        desired=m*countf - m*td
    elif(countf>tr+t): 
        desired = -m*countf+ (peak+m*(tr+t))
    if(countf >t2):
        countf=0; 
    if(desired<0):
        desired=0

def callback_throttle(data):
    global throttle
    throttle=data.linear.x

def callback_feedback(data):
    # conversion of odometry readings from quarternion to euler
    global desired, throttle
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)

    vel = (data.twist.twist.linear.x * math.cos(yaw) +
                         data.twist.twist.linear.y * math.sin(yaw))
    trap()
    vel_prof=Twist()
    vel_prof.linear.x=desired
    pub.publish(vel_prof)

    info_c=Point32()

    # x will contain velocity profile. y will contain the velocity response, z contains throttle
    info_c.x = desired
    info_c.y = vel
    info_c.z = throttle
    pub1.publish(info_c) 
    print "Published"
    print "Desired: " + str(desired)


def start():
    global pub,pub1
    rospy.init_node('controls',anonymous = True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) #publish the velocity profile onto the PID node
    pub1 = rospy.Publisher('info', Point32, queue_size=10) #for publishing the velocity profile, velocity response and throttle
    rospy.Subscriber("/pid_output", Twist, callback_throttle) #get throttle output from the PID node
    rospy.Subscriber("/odometry/filtered1", Odometry, callback_feedback) #get current velocity of the car
    rospy.spin()
if __name__ == '__main__':
	start()
         
