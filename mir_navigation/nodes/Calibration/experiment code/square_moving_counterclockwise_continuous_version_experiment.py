#! /usr/bin/env python

""" square_moving_counterclockwise.py : This program aims to control the mir_200 robot moving in a square path. Move by publishing velocity command cmd_vel;
    stop publishing velocity command by checking odometry info from /odom topic; track the real pose in simulation by checking info from the base_ground_pose_truth topic;
	track the real pose in experiment by using laser tracker.
    Written by Cui
"""

import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, copysign, sqrt, pow, pi
from nav_msgs.msg import Odometry
#For subscribing the topics odom and base_pose_ground_truth using message type Odometry
from tf.transformations import euler_from_quaternion

class SquareMovingCounterclockwise():
	"""class for moving mir_200 robot in counterclockwise direction with square path"""
	def __init__(self):
		#Give a name for node
		rospy.init_node('square_moving_counterclockwise', anonymous=False)
		#Set rospy to execute the shutdown function in this class when terminating this script
		rospy.on_shutdown(self.shutdown)
		#Set the frequency of checking odometry values, set it 2 times as publishing frequency of odometry in file diff_drive_controller.cpp
		rate = 100
		#Set the ROS rate variable equal to the rate variable
		self.r = rospy.Rate(rate)

		#Define variables for callback function of subscriber /odom
		self.x = 0
		self.y = 0
		self.theta = 0

		#Define variables for callback function of subscriber base_pose_ground_truth
		self.x_r = 0
		self.y_r = 0
		self.theta_r = 0

		#Define a list for write date file
		self.real_pose = []

		#Set parameters for square path
		self.translation_length = rospy.get_param("param_translation_length", 1.5)            # in meters
		self.rotation_angle = rospy.get_param("param_rotation_angle", radians(90))            # degree in radinas
		self.linear_velocity = rospy.get_param("param_linear_velocity", 0.1)                  # in meters per second (same as the theory and experiment, should as slow as possible)
		self.angular_velocity = rospy.get_param("param_angular_velocity", 0.1)                # in radians per second (should as slow as possible)
		self.translation_tolerance = rospy.get_param("param_translation_tolerance", 0)        # tolerance to control when stop the translation
		self.rotation_tolerance = rospy.get_param("param_rotation_tolerance", radians(0.00))  # see below
		# tolerance to control when stop the rotation and to compensate the different between odometry orientation and real pose orientation

		#Define a pulisher with topic cmd_vel for publishing velocity of robot
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		#Define a subscriber with topic odom for subscribing the wrong odometry of robot
		self.odom = rospy.Subscriber("/odom", Odometry, self.OdometryCallback)

		#Define a subschribe with topic base_pose_ground_truth for subscribing the real pose of robot
		self.pose = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.PoseCallback)

		#Set parameters for base_frame, odom_frame, map_frame
		#self.base_frame = rospy.get_param('base_frame', '/base_footprint')
		#self.odom_frame = rospy.get_param('odom_frame', '/odom_comb')
		#self.map_frame = rospy.get_param('map_frame', '/map')

		#Initialize the tf listener
		#self.tf_listener = tf.TransformListener()

		#check if the transform between /map and /base_footprint exists
		#try:
		#	self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
		#except (tf.Exception, tf.ConnectivityException, tf.LookupException):
		#	rospy.loginfo("Cannot find the tf transform between /map and /base_footprint")
		#	rospy.signal_shutdown("tf Exception")

		#Start moving robot with sqaure path
		choice = self.choose()

		if (choice == 1):
			self.state = self.MovingRobot(self.translation_length, self.rotation_angle, self.linear_velocity, self.angular_velocity, self.translation_tolerance, self.rotation_tolerance)
			if self.state == True:
				rospy.loginfo("sqaure path has been completed")
			else:
				rospy.loginfo("sqaure moving failed")
		elif (choice == 0):
			exit(0)
		
		while (choice != 'a'):
			choice = self.choose()
			if (choice == 1):
				self.state = self.MovingRobot(self.translation_length, self.rotation_angle, self.linear_velocity, self.angular_velocity, self.translation_tolerance, self.rotation_tolerance)
				if self.state == True:
					rospy.loginfo("sqaure path has been completed")
				else:
					rospy.loginfo("sqaure moving failed")
			elif (choice == 0):
				exit(0)

	#Define a function for choosing the options
	def choose(self):

		choice = 'a'
		rospy.loginfo("--------------------------------------------------------------------------")
		rospy.loginfo("choose 1: moving robot with square path in counterclockwise direction")
		rospy.loginfo("choose 0: quit")
		rospy.loginfo("please choose...")
		rospy.loginfo("--------------------------------------------------------------------------")
		choice = input()
		return choice

	#Define a function named MovingRobot for moving robot
	def MovingRobot(self, translation_length, rotation_angle, linear_velocity, angular_velocity, translation_tolerance, rotation_tolerance):

		#Initialize the real position variable as point type and get the real pose
		#real_position = Point()
		#(real_position, real_rotation) = self.get_real_odom()
		#self.real_pose.append(real_position.x)
		#self.real_pose.append(real_position.y)
		#self.real_pose.append(real_rotation)

		#rospy.loginfo("Real start pose from base_pose_ground_truth x, y, theta: %.3f, %.3f, %.3f", real_position.x, real_position.y, real_rotation*180/pi)

		#4 times translation and rotation of robot to achieve the square path
		for i in range(4):

			#Initialize the position variable as a point type
			position = Point()
			#Initialize the movement command, default values should be 0.
			move_cmd = Twist()

			#Set the linear velocity for forward translation
			move_cmd.linear.x = linear_velocity
			move_cmd.linear.y = 0
			move_cmd.linear.z = 0
			move_cmd.angular.x = 0
			move_cmd.angular.y = 0
			move_cmd.angular.z = 0

			#Get and print the starting pose of robot using get_wrong_odom function
			(position, rotation) = self.get_wrong_odom()
			rospy.loginfo("Pose estimation from odometry x, y, theta: %.3f, %.3f, %.3f", position.x, position.y, rotation*180/pi)

			x_start = position.x
			y_start = position.y

			#Initialize the distance variable for checking distance traveled
			distance = 0

			#Enter the loop to move robot forward (4m translation by default)
			while (distance + translation_tolerance) < translation_length and not rospy.is_shutdown():

				#Publish the Twist message as velocity command
				self.cmd_vel.publish(move_cmd)
				self.r.sleep()

				#Get the current pose of robot using get_odom function
				(position, rotation) = self.get_wrong_odom()
				#(real_position, real_rotation) = self.get_real_odom()

				#Compute the Euclidean distance from start until now
				distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

			#Print the pose estimation from odometry after each translation
			rospy.loginfo("Pose estimation from odometry x, y, theta: %.3f, %.3f, %.3f", position.x, position.y, rotation*180/pi)
			#rospy.loginfo("Real Pose from base_pose_ground_truth x, y, theta: %.3f, %.3f, %.3f", real_position.x, real_position.y, real_rotation*180/pi)
			
			#Stop the translation movement before rotation by publishing the zero value of velocity
			move_cmd.linear.x = 0
			move_cmd.linear.y = 0
			move_cmd.linear.z = 0
			move_cmd.angular.x = 0
			move_cmd.angular.y = 0
			move_cmd.angular.z = 0
			self.cmd_vel.publish(move_cmd)
			rospy.sleep(1)

			#Set the angular velocity for translation in counterclockwise direction
			move_cmd.linear.x = 0
			move_cmd.linear.y = 0
			move_cmd.linear.z = 0
			move_cmd.angular.x = 0
			move_cmd.angular.y = 0
			move_cmd.angular.z = angular_velocity

			#Track the last angle measured before starting the rotation
			last_angle = rotation

			#Initialize the angular variable for checking the angle traveled
			turn_angle = 0

			#Enter the loop to turn robot in counterclockwise direction (90 rotation by default)
			while abs(turn_angle - rotation_tolerance) < abs(rotation_angle) and not rospy.is_shutdown():

				#Publish the Twist message as velocity command
				self.cmd_vel.publish(move_cmd)
				self.r.sleep()

				#Get the current pose of robot using get_odom function
				(position, rotation) = self.get_wrong_odom()
				#(real_position, real_rotation) = self.get_real_odom()

				#Compute the rotation traveled from start until now
				changed_angle = self.normalize_angle(rotation - last_angle)

				turn_angle += changed_angle
				last_angle = rotation

			#Print the pose estimation from odometry after each rotation
			rospy.loginfo("Pose estimation from odometry x, y, theta: %.3f, %.3f, %.3f", position.x, position.y, rotation*180/pi)
			#rospy.loginfo("Real Pose from base_pose_ground_truth x, y, theta: %.3f, %.3f, %.3f", real_position.x, real_position.y, real_rotation*180/pi)

			#Stop the rotation movement before next translation by publishing the zero value of velocity
			move_cmd = Twist()
			move_cmd.linear.x = 0
			move_cmd.linear.y = 0
			move_cmd.linear.z = 0
			move_cmd.angular.x = 0
			move_cmd.angular.y = 0
			move_cmd.angular.z = 0
			self.cmd_vel.publish(move_cmd)
			rospy.sleep(1)

		#Compute the real end pose
		#(real_position, real_rotation) = self.get_real_odom()
		#self.real_pose.append(real_position.x)
		#self.real_pose.append(real_position.y)
		#self.real_pose.append(real_rotation)

		#Stop the robot after square path movement by publishing the zero value of velocity
		move_cmd = Twist()
		move_cmd.linear.x = 0
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0
		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
		move_cmd.angular.z = 0
		self.cmd_vel.publish(move_cmd)
		
		#rospy.loginfo("Real end pose from base_pose_ground_truth x, y, theta: %.3f, %.3f, %.3f", real_position.x, real_position.y, real_rotation*180/pi)

		#Write file to record the start and end of real pose, this file should contain 6 elements, 3 for starting x y rotation, 3 for ending x y rotation
		#with open("real_pose_counterclockwise.txt", 'w') as f:
		#	for j in self.real_pose:
		#		f.write(str(j) + '\n')

		return True

	#Define a function named get_real_odom to get the current pose from topic base_pose_ground_truth
	def get_real_odom(self):

		pos_r = Point()
		pos_r.x = self.x_r
		pos_r.y = self.y_r
		return (pos_r, self.theta_r)
	
	#Define a callback function for subscriber (topic: base_pose_ground_truth)
	def PoseCallback(self, pose_msg):

		self.x_r = pose_msg.pose.pose.position.x
		self.y_r= pose_msg.pose.pose.position.y
		rot_q_r = pose_msg.pose.pose.orientation
		(roll_r, pitch_r, self.theta_r) = euler_from_quaternion([rot_q_r.x, rot_q_r.y, rot_q_r.z, rot_q_r.w])	

	#Define a function get_wrong_odom to get the current pose which odometry thought it is right
	def get_wrong_odom(self):

		pos = Point()
		pos.x = self.x
		pos.y = self.y
		return (pos, self.theta)

	#Define a callback function for subscriber (topic: odom)
	def OdometryCallback(self, odom_msg):
		
		self.x = odom_msg.pose.pose.position.x
		self.y = odom_msg.pose.pose.position.y
		rot_q = odom_msg.pose.pose.orientation
		(roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
	#Define a function for normalizing the angle
	def normalize_angle(self, angle):

		res = angle
		while res > pi:
			res -= 2.0*pi
		while res < -pi:
			res += 2.0*pi
		return res

	#Define a shutdown function
	def shutdown(self):

		#Always stop the robot when shutting down the node or terminating the script
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)


if __name__ == '__main__':
	try:
		SquareMovingCounterclockwise()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Movement terminated")
