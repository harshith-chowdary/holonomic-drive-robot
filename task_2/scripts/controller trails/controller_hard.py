#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 2 of HolA Bot (HB) Theme (eYRC 2022-23).
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
# Filename:			controller.py
# Functions:
#				[ Comma separated list of functions in this file ]
# Nodes:			Add your publishing and subscribing node
	

################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

hola_x = 0
hola_y = 0
hola_theta = 0

x_goals = [350]
y_goals = [350]
theta_goals = [0.785]

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
	pass

def task2_goals_Cb(msg):
	global x_goals, y_goals, theta_goals
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################
	
	global hola_x, hola_y, hola_theta

	hola_x = msg.x
	hola_y = msg.y
	hola_theta = msg.theta
	

def inverse_kinematics(x, y, w, theta):

	front_alpha = 180*(PI/180) + theta
	left_alpha = 300*(PI/180) + theta
	right_alpha = 420*(PI/180) + theta

	arr = np.array([[math.cos(front_alpha), math.cos(left_alpha), math.cos(right_alpha)], [math.sin(front_alpha), math.sin(left_alpha), math.sin(right_alpha)], [1, 1, 1]])

	inverse_arr = np.linalg.pinv(arr)

	vel = [x, y, w]

	arr = np.dot(inverse_arr, vel)

	m = -200
	
	arr[0] = round(arr[0],6)*m
	arr[1] = round(arr[1],6)*m
	arr[2] = round(arr[2],6)*m

	print(-arr[0], -arr[1], -arr[2])

	return arr[0], arr[1], arr[2]

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################


def main():

	rospy.init_node('controller_node')

	rospy.loginfo('Hello Sir')

	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	#rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	vel_front = Wrench()
	vel_right = Wrench()
	vel_left = Wrench()

	vel_front.force.x = 0
	vel_right.force.x = 0
	vel_left.force.x = 0

	front_wheel_pub.publish(vel_front)
	right_wheel_pub.publish(vel_right)
	left_wheel_pub.publish(vel_left)

	time.sleep(5)

	speed = 1
		
	for i in range(0,len(x_goals)):
		
		x_d = x_goals[i]
		y_d = 500 - y_goals[i]
		theta_d = theta_goals[i]
		theta_diff = theta_d - hola_theta
		
		if theta_diff>0:
			if (theta_diff//PI)%2 == 0:
				theta_diff-= 2*PI*(theta_diff//PI)
			else:
				theta_diff = -PI + theta_diff%PI
		elif theta_diff<0:
			theta_diff = abs(theta_diff)
			if (theta_diff//PI)%2 == 0:
				theta_diff-= 2*PI*(theta_diff//PI)
			else:
				theta_diff = -PI + theta_diff%PI
			theta_diff*=-1
		
		print("theta_diff : ",theta_diff)

		theta_d = theta_diff + hola_theta
			
		hola_d = [x_d, y_d]
		hola_bot = [hola_x, hola_y]
		dist = 0
		
		if abs(x_d - hola_x) < 2.5 and abs(y_d - hola_y) < 2.5:
			rospy.loginfo("At location !!! Rotating ... ")
			angular_speed = 1
		else:
			dist = math.dist(hola_bot, hola_d)
			angular_speed = (theta_diff*speed)/dist
				
		print("Angular Speed   = ", angular_speed)
		print("Time Linear     = ", dist/speed)
		
		if (theta_diff ==0):
			print("Time Rotational = 0")
		else:
			print("Time Rotational = ", theta_diff/angular_speed)
		
		#
		# 
		# Control Loop goes here
		# For maintaining control loop rate.
		rate = rospy.Rate(100)

		while not rospy.is_shutdown():

			# Find error (in x, y and theta) in global frame
			# the /odom topic is giving pose of the robot in global frame
			# the desired pose is declared above and defined by you in global frame
			# therefore calculate error in global frame
			x_err_gf = x_d - hola_x
			y_err_gf = y_d - hola_y
			theta_err = theta_d - hola_theta
			
			f1=0
			f2=0

			if (abs(theta_err) < 5*PI/180):
				angular_speed = 0
				f1 = 1
				
			if (abs(x_err_gf) < 5 and abs(y_err_gf) < 5):
				if (f1==0 and theta_err>0):
					rospy.loginfo("Reached location !!! Rotating ... Anti Clock")
				elif(f1==0):
					rospy.loginfo("Reached location !!! Rotating ... Clock")
				f2 = 1
				if not f1:
					if (abs(theta_err) < 10*PI/180):
						if (theta_err>0):
							angular_speed = 0.5
						else:
							angular_speed = -0.5
				
			if (f1*f2):
				vel_front.force.x = 0
				vel_right.force.x = 0
				vel_left.force.x = 0
				front_wheel_pub.publish(vel_front)
				right_wheel_pub.publish(vel_right)
				left_wheel_pub.publish(vel_left)
				break

			# (Calculate error in body frame)
			# But for Controller outputs robot velocity in robot_body frame, 
			# i.e. velocity are define is in x, y of the robot frame, 
			# Notice: the direction of z axis says the same in global and body frame
			# therefore the errors will have have to be calculated in body frame.
			# 
			# This is probably the crux of Task 1, figure this out and rest should be fine.
			cos_theta = math.cos(hola_theta)
			sin_theta = math.sin(hola_theta)
			
			if not f2:
				x_err_bf = x_err_gf*cos_theta + y_err_gf*sin_theta
				y_err_bf = y_err_gf*cos_theta - x_err_gf*sin_theta
				'''
				if (x_err_bf < 0.05):
					alpha = PI/2
				else:
					alpha = math.atan(y_err_bf/x_err_bf)
				'''
				alpha = math.atan2(y_err_bf, x_err_bf)
				
				vel_x = speed*math.cos(alpha)
				vel_y = speed*math.sin(alpha)

				if not f1:
					hola_bot = [hola_x, hola_y]
					dist = math.dist(hola_bot, hola_d)
					try:
						angular_speed = (100/50) * (theta_err*speed)/dist
					except ZeroDivisionError:
						if (theta_err>0):
							angular_speed = 1
						else:
							angular_speed = -1

			else:
				vel_x = 0
				vel_y = 0

			if not x_err_gf:
				vel_x = 0
			if not y_err_gf:
				vel_y = 0
				
			# Finally implement a P controller 
			# to react to the error with velocities in x, y and theta.
			vel_z = angular_speed
		
			# Safety Check
			# make sure the velocities are within a range.
			# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
			# we may get away with skipping this step. But it will be very necessary in the long run.

			front, left, right = inverse_kinematics(vel_x, -vel_y, vel_z, hola_theta)

			vel_front.force.x = front
			vel_left.force.x = left
			vel_right.force.x = right

			front_wheel_pub.publish(vel_front)
			left_wheel_pub.publish(vel_left)
			right_wheel_pub.publish(vel_right)

			rate.sleep()
		#
		
		front_wheel_pub.publish(vel_front)
		right_wheel_pub.publish(vel_right)
		left_wheel_pub.publish(vel_left)
		
		print("Accomplished Successfully !!!")
		
		time.sleep(3)

	rospy.spin()

	'''
	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	#	   

		
	while not rospy.is_shutdown():
		
		# Calculate Error from feedback

		# Change the frame by using Rotation Matrix (If you find it required)

		# Calculate the required velocity of bot for the next iteration(s)
		
		# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

		# Apply appropriate force vectors

		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)

		rate.sleep()

	############################################
	'''

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass