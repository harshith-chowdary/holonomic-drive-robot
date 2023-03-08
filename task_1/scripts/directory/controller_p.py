#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type : Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type : Odometry
from nav_msgs.msg import Odometry

# for finding sin() and cos()
import math

# for delaying each test case by 1 sec
import time

# Odometry is given as a quaternion, but for controller we'll need to find the orientation theta by converting to eular angle
from tf.transformations import euler_from_quaternion

# auto-evaluator
from geometry_msgs.msg import PoseArray

# constants
PI = 22/7

# Global Variables
hola_x = 0
hola_y = 0
hola_theta = 0

x_goals = []
y_goals = []
theta_goals = []

we_r_there = False

def task1_goals_Cb(msg):
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
		
def odometryCb(msg):
	global hola_x, hola_y, hola_theta
	
	# Write your code to take the msg and update the three variables
	
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	quaternion_bot = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	eular_bot = euler_from_quaternion(quaternion_bot, axes='sxyz')
	'''
	roll = eular_bot[0]
	pitch = eular_bot[1]
	yaw = eular_bot[2]
	'''
	hola_theta = eular_bot[2]
	
def main():
	# Initialze Node
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
	
	rospy.init_node('controller', anonymous = True)
	
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively
	
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
	
	# declare that the node subscribes to task1_goals along with the other declarations of publishing and subscribing
	rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
	
	rospy.Subscriber('/odom', Odometry, odometryCb)
	
	vel = Twist()
	# Initialise the required variables to 0
	vel.linear.x = 0
	vel.linear.y = 0
	vel.angular.z = 0
	
	pub.publish(vel)
	
	time.sleep(5)
	
	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
	'''
	x_goals = [1, -1, -1, 1, 0]
	y_goals = [1, 1, -1, -1, 0]
	theta_goals = [PI/4, -PI, PI, -PI/4, 0]
	'''
	speed = 1
		
	for i in range(0,len(x_goals)):
		
		x_d = x_goals[i]
		y_d = y_goals[i]
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
			
		hola_d = [x_d, y_d]
		hola_bot = [hola_x, hola_y]
		dist = 0
		
		if (hola_bot == hola_d):
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
		
		x = 1
		y = 1
		w = 1
		Kp = 0.5
		
		while not rospy.is_shutdown():

			# Find error (in x, y and theta) in global frame
			# the /odom topic is giving pose of the robot in global frame
			# the desired pose is declared above and defined by you in global frame
			# therefore calculate error in global frame
			x_err_gf = x_d - hola_x
			y_err_gf = y_d - hola_y
			theta_err = theta_d - hola_theta
			
			v_x = x_err_gf*Kp*x
			v_y = y_err_gf*Kp*y
			v_w = theta_err*Kp*w
			
			if (abs(theta_err) < PI/180):
				w = 0
				
			if (abs(x_err_gf)<0.05):
				v_x_bf = 0
				x = 0
				
			if (abs(y_err_gf)<0.05):
				v_y_bf = 0
				y = 0
			
			# (Calculate error in body frame)
			# But for Controller outputs robot velocity in robot_body frame, 
			# i.e. velocity are define is in x, y of the robot frame, 
			# Notice: the direction of z axis says the same in global and body frame
			# therefore the errors will have have to be calculated in body frame.
			# 
			# This is probably the crux of Task 1, figure this out and rest should be fine.
			
			cos_theta = math.cos(hola_theta)
			sin_theta = math.sin(hola_theta)
			
			if x:
				v_x_bf = v_x*cos_theta + v_y*sin_theta
			
			if y:	
				v_y_bf = v_y*cos_theta - v_x*sin_theta
				
			# Finally implement a P controller 
			# to react to the error with velocities in x, y and theta.
		
			# Safety Check
			# make sure the velocities are within a range.
			# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
			# we may get away with skipping this step. But it will be very necessary in the long run.

			vel.linear.x = v_x_bf
			vel.linear.y = v_y_bf
			vel.angular.z = v_w

			pub.publish(vel)
			rate.sleep()
		#
		
		pub.publish(vel)
		
		print("Accomplished Successfully !!!")
		
		time.sleep(1)


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
