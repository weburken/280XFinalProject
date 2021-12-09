#! /usr/bin/env python2.7 
# we need 2.7 to import tf

import rospy # import rospy, a ROS-python wrapper 
from turtlesim.msg import Pose # import turtlesim message package. Pose is the daya type used to store x,y coordinates
from math import pow, atan2, sqrt, pi # import auxiliary math functions
from sensor_msgs.msg import LaserScan # import sensor message package to utilize the Lidar sensor
from geometry_msgs.msg import Twist  # import geometry message package. Twist is the data type used to describe linear and angular velocities
from nav_msgs.msg import Odometry # import navigation message package. Odometry is the data type to extract turtlebot's position in quaternions
import tf
from tf.transformations import euler_from_quaternion #import transformation package, which allows us to convert from quaternions to eulerian coordinates
import time
class TurtleBot:

	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)

		# Publisher which will publish to the topic 'robot2/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('robot4/cmd_vel', Twist, queue_size=10)

		# A subscriber to the topic '/odom'. self.update_pose is called when a message of type Pose is received.
		self.pose_subscriber = rospy.Subscriber('robot4/odom', Odometry, self.update_pose)
		self.laser_suscriber = rospy.Subscriber('robot4/scan', LaserScan, self.update_scan)
		# Create a pose object attributed to turtlebot
		self.odom = Odometry
		self.goal_pose = Pose()
		self.velGoal = Twist()
		# Set the rate at which we publish/suscribe in Hz
		self.rate = rospy.Rate(10)
		# We need a short pause to allow self.pose to suscribe from the topic and accurate display turtlebots pose
		rospy.sleep(0.5)
		print('Initiliazing at x:{}, y:{}'.format(self.pos_x, self.pos_y))

		self.distance_error = 0
		self.previous_distance_error = 0
		self.sum_distance_error = 0

		self.angular_error = 0
		self.previous_angular_error = 0
		self.sum_angular_error = 0

	def update_pose(self, data):
		"""Callback function that is called when a new message of type Pose is received by the pose_subscriber."""
		self.odom = data

		self.pos_x = round(self.odom.pose.pose.position.x, 4)
		self.pos_y = round(self.odom.pose.pose.position.y, 4)
		# convert quaternion coordinates in the form of (x,y,z,w) to eulerian coordinates (roll, pitch, yaw)
		self.roll,self.pitch,self.yaw = euler_from_quaternion((self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, \
															   self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w))
		# we only use yaw angle since in this case, turtlebot is constrained to rotation around the z-axis
		self.theta = self.yaw

	def update_scan(self, data):
		"""Callback function that is called when a new message of type LaserScan is received by the laser_subscriber."""

		# updates distance to any obstacle in front of turtlebot
		self.front_laser = data.ranges[0]

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.x - self.pos_x), 2) + pow((goal_pose.y - self.pos_y), 2))

	def linear_vel(self, goal_pose, k_p, k_i, k_d):
		# Store previous error in a variable
		self.previous_distance_error = self.distance_error
		# Update the distance error with respect to current pose
		self.distance_error = self.euclidean_distance(goal_pose)
		# Sum cumulative distance error over time
		self.sum_distance_error += self.distance_error

		return k_p*self.distance_error + k_i*self.sum_distance_error + k_d*(self.distance_error-self.previous_distance_error)

	def steering_angle(self, goal_pose):
		"""Steering angle between current pose and the goal"""
		angle = atan2(goal_pose.y - self.pos_y, goal_pose.x - self.pos_x)
		return angle

	def angular_vel(self, goal_pose, k_p, k_i, k_d):
		# Store previous error in a variable
		self.previous_angular_error = self.angular_error
		# Update the distance error with respect to current pose
		self.angular_error = self.steering_angle(goal_pose) - self.theta
		# Sum cumulative distance error over time
		self.sum_angular_error += self.angular_error

		return k_p*(self.angular_error) + k_i*(self.sum_angular_error) + k_d*(self.angular_error - self.previous_angular_error)
	def update_Leadpose(self, data):#TODO: CHeck
		"""Callback function that is called when a new message of type Pose is received by the pose_subscriber."""
		self.goal_pose.x = data.pose.pose.position.x
		self.goal_pose.y = data.pose.pose.position.y
		#print("xPos: ", self.goal_pose.x, ", yPos: ", self.goal_pose.y)

	def update_LeaderVel(self, data):#TODO: CHeck
		"""Callback function that is called when a new message of type Pose is received by the pose_subscriber."""
		self.velGoal = data
		print("GoalVel: ", self.velGoal.linear.x, ", Actual: ")

	def move2goal(self, x, y, xp, xi, xd, ap, ai, ad, finList):
		start = time.time()
		"""Moves the turtlebot to the goal."""
		# Creates a pose object
		# Get the input from the user.
		link_p = xp
		link_i = xi
		link_d = xd
		angk_p = ap
		angk_i = ai
		angk_d = ad

		follow_subscriber = rospy.Subscriber('robot3/odom', Odometry, self.update_Leadpose)
		velocity_subscriber = rospy.Subscriber('robot3/cmd_vel', Twist, self.update_LeaderVel)
		# Insert a number slightly greater than 0 (e.g. 0.01).
		distance_tolerance = 0.75#float(input("Set your tolerance for goal: "))

		# Instantiate Twist object to send mesg to turtlebot
		vel_msg = Twist()

		# feedback loop to keep sending control signal while distance > tolerance
		while self.euclidean_distance(self.goal_pose) >= distance_tolerance or self.velGoal.linear.x > 0.01:
			if self.euclidean_distance(self.goal_pose) >= distance_tolerance:
				if self.front_laser < 0.01:
					print('Obstacle detected in front, modify code below to avoid collision')
					vel_msg.linear.x = self.linear_vel(self.goal_pose, link_p*self.front_laser, link_i*self.front_laser, link_d*self.front_laser)
					vel_msg.angular.z = self.angular_vel(self.goal_pose, angk_p*self.front_laser, angk_i*self.front_laser, angk_d*self.front_laser)
				else:
					# Linear velocity in the x-axis.
					vel_msg.linear.x = self.linear_vel(self.goal_pose, link_p, link_i, link_d)

					# Angular velocity in the z-axis.
					vel_msg.angular.z = self.angular_vel(self.goal_pose, angk_p, angk_i, angk_d)

				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)

				# Publish at the desired rate.
				self.rate.sleep()
			else:
				vel_msg.linear.x = self.velGoal.linear.x

					# Angular velocity in the z-axis.
				vel_msg.angular.z = self.angular_vel(self.goal_pose, angk_p, angk_i, angk_d)

				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)

				# Publish at the desired rate.
				self.rate.sleep()

		# Stopping our robot after destination is reached
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		end = time.time()
		finList.append((end-start))
		print('Arrived at location x:{}, y:{}'.format(self.pos_x, self.pos_y))

def run(x,y,xp, xi, xd, ap, ai, ad, finList):
	try:
		x = TurtleBot()
		x.move2goal(x,y,xp, xi, xd, ap, ai, ad, finList)
	except rospy.ROSInterruptException:
		pass
if __name__ == '__main__':
	try:
		finList =[]
		x = TurtleBot()
		x.move2goal(-3,5, 0.25, 0,0,5,0,10, finList)


	except rospy.ROSInterruptException:
		pass
