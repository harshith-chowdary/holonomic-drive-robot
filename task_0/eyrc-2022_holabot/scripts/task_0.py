#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			eYRC#HB#4266
# Author List:		Harshith Chowdary Meduri, Gitanjali Gupta, Bibhudatta Bhanja, Priyanshu Sahu
# Filename:			task_0.py
# Functions:
# 					[ Comma separated list of functions in this file ]
# Nodes:		    Publisher: pub,Pub; Subscriber: sub,Sub


####################### IMPORT MODULES #######################
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import traceback
##############################################################


def callback(data):
	
	if (type(data)==String):
		rospy.loginfo("My turtleBot is: %s",data.data)
	elif (type(data)==Twist):
		rospy.loginfo("@ Linear Velocity = %.2f and Angular Velocity = %.2f",data.linear.x,data.angular.z)

def main():

	rospy.init_node('turtlesim_node', anonymous = True)
	Pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1)
	pub = rospy.Publisher('/turtle1/cmd_status', String, queue_size = 1)
	rospy.Subscriber('/turtle1/cmd_status', String, callback)
	rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
	
	vel = Twist()
	
	radius = 1
	angular_speed = 0.1
	linear_speed = angular_speed*radius
	
	vel.linear.x = linear_speed
	vel.linear.y = 0
	vel.linear.z = 0
	
	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = angular_speed
	
	current_distance = 0.000000
	half_circle = PI*radius
	t0 = rospy.Time.now().to_sec()
	
	string = "Moving in Circle"
	c = 100001
	while (current_distance < half_circle):
		
		c+=1
		
		if (c%100000==0):		
			pub.publish(string)
			Pub.publish(vel)
		
		t1 = rospy.Time().now().to_sec()
		current_distance = linear_speed*(t1-t0)
		
	pub.publish(string)
	Pub.publish(vel)
	
	current_angle = 0.000000
	to_angle = 90
	angle = to_angle*PI/180
	vel.linear.x = 0
	t0 = rospy.Time.now().to_sec()
	
	string = "Rotating"
	c = 100001
	while (current_angle < angle):
		
		c+=1
		
		if (c%100000==0):		
			pub.publish(string)
			Pub.publish(vel)
		
		t1 = rospy.Time().now().to_sec()
		current_angle = angular_speed*(t1-t0)
		
	pub.publish(string)
	Pub.publish(vel)
	vel.angular.z = 0
	vel.linear.x = linear_speed
	
	
	current_distance = 0
	distance = 2*radius
	t0 = rospy.Time().now().to_sec()
	
	string = "Moving Straight"
	c = 100001
	while (current_distance < distance):
		
		c+=1
		
		if (c%100000==0):		
			pub.publish(string)
			Pub.publish(vel)
		
		t1 = rospy.Time().now().to_sec()
		current_distance = linear_speed*(t1-t0)
		
	pub.publish(string)
	Pub.publish(vel)
	vel.linear.x = 0
	
	rospy.spin()

################# ADD GLOBAL VARIABLES HERE #################

PI = 22/7

##############################################################


################# ADD UTILITY FUNCTIONS HERE #################


##############################################################


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
