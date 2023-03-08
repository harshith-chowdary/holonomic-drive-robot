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
# Filename:			feedback.py
# Functions:
#				[ Comma separated list of functions in this file ]
# Nodes:			aruco_publisher


######################## IMPORT MODULES ##########################

from re import T
import numpy				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()
pi = math.pi

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use OpenCV to find ARUCO MARKER from the IMAGE
	#	-> You are allowed to use any other library for ARUCO detection, 
	#        but the code should be strictly written by your team and
	#	   your code should take image & publish coordinates on the topics as specified only.  
	#	-> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
	#	-> Observe the accuracy of aruco detection & handle every possible corner cases to get maximum scores !

	############################################

	#cv2.imshow("hola_bot", current_frame)
	#cv2.waitKey(1)
	total_markers = 250
	key = getattr(cv2.aruco,f'DICT_{4}X{4}_{total_markers}')
	arucoDict = cv2.aruco.Dictionary_get(key)
	arucoParams = cv2.aruco.DetectorParameters_create()
	corners, ids, _ = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)
	draw = cv2.aruco.drawDetectedMarkers(current_frame, corners, ids)

	corners = numpy.asarray(corners)
	corners = corners.reshape((4,2))

	(top_left, top_right, bottom_right, bottom_left) = corners
	
	top_left = [int(top_left[0]), int(top_left[1])]
	top_right = [int(top_right[0]), int(top_right[1])]
	bottom_right = [int(bottom_right[0]), int(bottom_right[1])]
	bottom_left = [int(bottom_left[0]), int(bottom_left[1])]

	#1
	#'''
	top = numpy.add(top_left, top_right)
	bottom = numpy.add(bottom_left, bottom_right)
	center = numpy.add(top, bottom)
	center = numpy.dot(center, 0.25)
	#'''
	#2
	'''
	center = numpy.add(top_left, bottom_right)
	center = center * 0.5
	'''
	#3
	'''
	center = numpy.add(top_right, bottom_left)
	center = center * 0.5
	'''

	aruco_msg.x = center[0]
	aruco_msg.y = center[1]
	rospy.loginfo(center)

	current_frame = cv2.circle(current_frame, center.astype(int), 1, (255, 100, 100), -1)
	cv2.imshow("overhead_camera_vision", current_frame)
	cv2.waitKey(1)

	#rospy.loginfo(top_left)
	#rospy.loginfo(top_right)

	if (top_left[0]==top_right[0]):
		if (top_left[1]<top_right[1]):
			angle = pi/2
		else:
			angle = -pi/2
	elif (top_left[1]==top_right[1]):
		if (top_left[0]<=top_right[0]):
			angle = 0
		else:
			angle = pi
	else:

		angle = math.atan2(top_right[1] - top_left[1], top_right[0] - top_left[0])

		'''
		angle = math.atan2(abs(top_right[1] - top_left[1]), abs(top_right[0] - top_left[0]))
		if (top_right[0]<top_left[0]):
			if (top_right[1]>top_left[1]):
				angle = pi - angle
			else:
				angle = -pi + angle
		'''

	angle*=-1
	aruco_msg.theta = angle
	rospy.loginfo(angle)

	aruco_publisher.publish(aruco_msg)
	
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()