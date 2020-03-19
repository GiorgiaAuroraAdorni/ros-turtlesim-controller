#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from math import pow, atan2, sqrt, sin, cos
 
 
class TurtleBot:
	def __init__(self):
	# Creates a node with name 'turtlebot_controller' and make sure it is a
	# unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

		self.srv_setpen = rospy.ServiceProxy('turtle1/set_pen', SetPen)

		# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
		# when a message of type Pose is received.
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

		self.pose = Pose()
		self.rate = rospy.Rate(10)

	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is received by the subscriber."""
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)
 
	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.x - self.pose.x), 2) +
					pow((goal_pose.y - self.pose.y), 2))
 
	def linear_vel(self, goal_pose, constant=1.5):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * self.euclidean_distance(goal_pose)
 
	def steering_angle(self, goal_pose):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
 
	def angular_vel(self, goal_pose, constant=6):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * (self.steering_angle(goal_pose) - self.pose.theta)
		# return atan2(sin(self.steering_angle(goal_pose) - self.pose.theta), cos(self.steering_angle(goal_pose) - self.pose.theta))

	def move2goal(self):
		"""Moves the turtle to the goal."""
		# U
		# x = [1, 1, 2, 3, 3]
		# y = [8, 5, 4, 5, 8]

		# S
		# [7, 4, 7, 4]
		# [8, 6.67, 5.34, 4]

		# I
		# [9, 9]
		# [4, 8]

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
				# Porportional controller.
				# https://en.wikipedia.org/wiki/Proportional_control
	 
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

			self.srv_setpen(1,0,0,0,0)
	 
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