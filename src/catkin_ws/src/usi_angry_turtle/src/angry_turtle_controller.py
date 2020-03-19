#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import random
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import *
from turtlesim.srv import *
from std_srvs.srv import *
from math import pow, atan2, sqrt, sin, cos
 
 
class TurtleBot:
	def __init__(self):
	# Creates a node with name 'turtlebot_controller' and make sure it is a
	# unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

		# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
		# when a message of type Pose is received.
		self.pose_t1_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_pose)
		self.pose_t2_subscriber = rospy.Subscriber('/turtleTarget/pose', Pose, self.turtle2_pose)

		self.pose_t1 = Pose()
		self.pose_t2 = Pose()
		self.rate = rospy.Rate(10)

		self.srv_setpen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
		self.spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
		self.kill_turtle = rospy.ServiceProxy('/kill', Kill)
		self.clear = rospy.ServiceProxy('/clear', Empty)

	def turtle1_pose(self, data):
		"""Callback function which is called when a new message of type Pose is received by the subscriber."""
		self.pose_t1 = data
		self.pose_t1.x = round(self.pose_t1.x, 4)
		self.pose_t1.y = round(self.pose_t1.y, 4)
		self.pose_t1.theta = round(self.pose_t1.theta, 4)

	def turtle2_pose(self, data):
		"""Callback function which is called when a new message of type Pose is received by the subscriber."""
		self.pose_t2 = data
		self.pose_t2.x = round(self.pose_t2.x, 4)
		self.pose_t2.y = round(self.pose_t2.y, 4)
 
	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.x - self.pose_t1.x), 2) +
					pow((goal_pose.y - self.pose_t1.y), 2))
 
	def linear_vel(self, goal_pose, constant=1.5):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * self.euclidean_distance(goal_pose)
 
	def steering_angle(self, goal_pose):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return atan2(goal_pose.y - self.pose_t1.y, goal_pose.x - self.pose_t1.x)
 
	def angular_vel(self, goal_pose, constant=6):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * (self.steering_angle(goal_pose) - self.pose_t1.theta)
		# return atan2(sin(self.steering_angle(goal_pose) - self.pose_t1.theta), cos(self.steering_angle(goal_pose) - self.pose_t1.theta))

	def spawn_new_turtle(self):
		"""" Create a new turtle"""
		x = random.randint(4, 4)
		y = random.randint(4, 4)
		self.spawn_turtle(x, y, 0, "turtleTarget")

	def become_angry(self):
		print 'killing turtle target'
		try:
			self.kill_turtle("turtleTarget")
		except:
			pass
		self.clear()
		sys.exit()

	def move2goal(self):
		"""Moves the turtle to the goal."""
		self.spawn_new_turtle()

		tolerance = 1

		p = [Pose(x=1, y=8), Pose(x=1, y=5), Pose(x=2, y=4), Pose(x=3, y=5), Pose(x=3, y=8), 
			 Pose(x=7, y=8), Pose(x=4, y=6.67), Pose(x=7, y=5.34), Pose(x=4, y=4), 
			 Pose(x=9, y=4), Pose(x=9, y=8), Pose(x=0, y=0)]
		pen_offline = [0, 5, 9, 11]

		distance_tolerance = 0.1

		for idx, goal_pose in enumerate(p):
	 		if idx in pen_offline:
	 			self.srv_setpen(0,0,0,0,1)

			vel_msg = Twist()
	 		
	 		while self.euclidean_distance(goal_pose) >= distance_tolerance:
				# Porportional controller
				# Linear velocity in the x-axis.
				vel_msg.linear.x = self.linear_vel(goal_pose)
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0
	 
				 # Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = self.angular_vel(goal_pose)
	 
				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)
	 
				# Publish at the desired rate.
				self.rate.sleep()

				# See how far are the turtles	
				if self.euclidean_distance(self.pose_t2) < tolerance:
					self.become_angry()

			self.srv_setpen(255,255,255,3,0)
	 
		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
 
		# If we press control + C, the node will stop.
		rospy.spin()
 
if __name__ == '__main__':
	try:
		x = TurtleBot()
		x.move2goal()
	except rospy.ROSInterruptException:
		pass