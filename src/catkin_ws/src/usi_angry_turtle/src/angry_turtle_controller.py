#!/usr/bin/env python
import random
import sys
from math import pow, atan2, sqrt, sin, cos

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import *
from turtlesim.msg import *
from turtlesim.srv import *


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

        self.srv_setpen1 = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.srv_setpen2 = rospy.ServiceProxy('/turtleTarget/set_pen', SetPen)
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

    def linear_vel(self, goal_pose, constant=3):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose_t1.y, goal_pose.x - self.pose_t1.x)

    def angular_vel(self, goal_pose, constant=12):
        # return constant * (self.steering_angle(goal_pose) - self.pose_t1.theta)
        return constant * atan2(sin(self.steering_angle(goal_pose) - self.pose_t1.theta),
                                cos(self.steering_angle(goal_pose) - self.pose_t1.theta))

    def spawn_new_turtle(self):
        """" Create a new turtle"""
        offender_x = random.randint(1, 12)
        offender_y = random.randint(1, 12)
        self.spawn_turtle(offender_x, offender_y, 0, "turtleTarget")

    def become_angry(self):
        print 'pursuing the offender turtle'
        vel_msg = Twist()

        while self.euclidean_distance(self.pose_t2) >= 0.1:
            # Porportional controller
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(self.pose_t2)

        goal_pose = Pose()
        goal_pose.x = self.pose_t1.x + m * cos(self.pose_t1.theta)
        goal_pose.y = self.pose_t1.y + m * sin(self.pose_t1.theta)

        while self.euclidean_distance(self.pose_t1) >= 0.1:
            self.move_to_goal(goal_pose)

            # Publish at the desired rate.
            self.rate.sleep()

        print 'killing turtle target'
        try:
            self.kill_turtle("turtleTarget")
        except:
            pass
        self.clear()
        sys.exit()

    def move_to_goal(self, goal_pose):
        """

        :param goal_pose:
        """
        # Proportional (P) controller
        self.vel_msg.linear.x = self.linear_vel(goal_pose)    # Linear velocity in the x-axis.
        self.vel_msg.angular.z = self.angular_vel(goal_pose)  # Angular velocity in the z-axis.

        self.velocity_publisher.publish(self.vel_msg)

        # TODO move to method
        self.rate.sleep()
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException

    def rotate_to_goal(self, goal_pose):
        """

        :param goal_pose:
        """
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = self.angular_vel_rot(goal_pose)

        self.velocity_publisher.publish(self.vel_msg)

        self.rate.sleep()

    def main(self):
        """Moves the turtle to the goal."""
        # TODO
        #  - [ ] Add rotation while the turtle writes 'USI'
        #  - [ ] Add turtles
        #       - [ ] Add automatic/random movement of the spawned turtles
        #       - [ ] Move the hunter to the closer turtle
        #  - [ ] Add loop the enable the turtle writing again from the beginning ignoring the other turtles: state
        #        machine WRITING, ANGRY, RETURNING.
        #  - [ ] The angry turtle tries to look ahead m meter in front of the offender to intercept it.
        #        M is directly proportional to the current speed times the current distance
        #  - [ ] Eliminate the offender if is closer than a threshold

        self.spawn_new_turtle()

    self.srv_setpen2(0, 0, 0, 0, 1)

    tolerance = 2

        self.go_to_initial_position()
        self.stop_walking()

        # If we press control + C, the node will stop.
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.main()
    except rospy.ROSInterruptException:
        pass
