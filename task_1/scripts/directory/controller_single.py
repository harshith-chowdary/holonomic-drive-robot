#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type : Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type : Odometry
from nav_msgs.msg import Odometry

# for finding sin() and cos()
import math

# Odometry is given as a quaternion, but for controller we'll need to find the orientation theta by converting to eular angle
from tf.transformations import euler_from_quaternion

# constants
PI = 22/7

# Global Variables
hola_x = 0
hola_y = 0
hola_theta = 0

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
	sub = rospy.Subscriber('/odom', Odometry, odometryCb)
	
	vel = Twist()
	# Initialise the required variables to 0
	vel.linear.x = 0
	vel.linear.y = 0
	vel.angular.z = 0
	
	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
	
	x_goals = [1, -1, -1, 1, 0]
	y_goals = [1, 1, -1, -1, 0]
	theta_goals = [PI/4, 3*PI/4, -3*PI/4, -PI/4, 0]
	
	
	speed = 1
		
	x_d = 0
	y_d = 0
	theta_d = PI/4
	
	if theta_d>0:
		if (theta_d//PI)%2 == 0:
			theta_d-= 2*PI*(theta_d//PI)
		else:
			theta_d = -PI + theta_d%PI
	elif theta_d<0:
		theta_d = abs(theta_d)
		if (theta_d//PI)%2 == 0:
			theta_d-= 2*PI*(theta_d//PI)
		else:
			theta_d = -PI + theta_d%PI
		theta_d*=-1
		
	hola_d = [x_d, y_d]
	hola_bot = [hola_x, hola_y]
	theta_diff = theta_d - hola_theta
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
		
		if (abs(theta_err) < PI/180):
			angular_speed = 0
			f1 = 1
		else:
			hola_bot = [hola_x, hola_y]
			dist = math.dist(hola_bot, hola_d)
			try:
				angular_speed = (100/50) * (theta_err*speed)/dist
			except ZeroDivisionError:
				angular_speed = 1*abs(theta_err)/theta_err
			
		if abs(x_err_gf) < 0.05 and abs(y_err_gf) < 0.05:
			f2 = 1
			if not f1:
				angular_speed = abs(theta_err)/theta_err
			
		if (f1*f2):
			vel.linear.x = 0
			vel.linear.y = 0
			vel.angular.z = 0
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
		else:
			vel_x = 0
			vel_y = 0
			
		# Finally implement a P controller 
		# to react to the error with velocities in x, y and theta.
		vel_z = angular_speed
	
		# Safety Check
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run.

		vel.linear.x = vel_x
		vel.linear.y = vel_y
		vel.angular.z = vel_z

		pub.publish(vel)
		rate.sleep()
	#
	
	pub.publish(vel)
	
	print("Accomplished Successfully !!!")


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
