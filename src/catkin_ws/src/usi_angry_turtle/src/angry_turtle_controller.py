#!/usr/bin/env python
import random
from math import *
from threading import Thread

import rospy
from geometry_msgs.msg import *
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
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.turtle_pose)

        self.pose = Pose()
        self.pose_target_turtles = dict()

        self.rate = rospy.Rate(10)

        self.vel_msg = Twist()

        self.PEN_ON = SetPenRequest(255, 255, 255, 3, 0)
        self.PEN_OFF = SetPenRequest(0, 0, 0, 0, 1)

        self.srv_setpen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)

        self.kill_turtle = rospy.ServiceProxy('/kill', Kill)
        self.clear = rospy.ServiceProxy('/clear', Empty)

    def turtle_pose(self, data):
        """
        Callback function which is called when a new message of type Pose is received by the subscriber.
        :param data:
        """
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

    def turtle_target_pose(self, data, name):
        """
        Callback function which is called when a new message of type Pose is received by the subscriber.
        :param data:
        """
        data.x = round(data.x, 4)
        data.y = round(data.y, 4)
        data.linear_velocity = round(data.linear_velocity, 4)

        self.pose_target_turtles[name] = data

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
        thread = Thread(target=targets_controller_thread, args=[self])
        thread.start()

        self.go_to_initial_position()
        self.stop_walking()

        # If we press control + C, the node will stop.
        rospy.spin()


def targets_controller_thread(angry_turtle):
    # FIXME: spawn new turtles every n seconds
    total_turtles = 2

    spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)

    target_velocity_publisher = dict()

    rate = rospy.Rate(10)

    for t in range(total_turtles):
        name = 'turtleTarget' + str(t)

        offender_x = random.randint(1, 12)
        offender_y = random.randint(1, 12)
        spawn_turtle(offender_x, offender_y, 0, name)

        rospy.Subscriber('/%s/pose' % name, Pose, angry_turtle.turtle_target_pose, name)
        target_velocity_publisher[name] = rospy.Publisher('/%s/cmd_vel' % name, Twist, queue_size=10)

        rospy.wait_for_service('/%s/set_pen' % name)
        set_pen = rospy.ServiceProxy('/%s/set_pen' % name, SetPen)
        set_pen(angry_turtle.PEN_OFF)

    #  TODO Add automatic/random movement of the spawned turtles
    while not rospy.is_shutdown():
        for t in range(total_turtles):
            name = 'turtleTarget' + str(t)

            # The turtle walk randomly if it is not a teleoperated turtle.
            vel_msg = Twist()
            vel_msg.linear.x = 20 - random.random() * 30
            vel_msg.angular.z = 20 - random.random() * 16
            target_velocity_publisher[name].publish(vel_msg)
            # TODO check if is shutdown and call exception
            rate.sleep()

    # TODO when all the target are dead
    # clearStage()
    # sys.exit()


    pass


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.main()
    except rospy.ROSInterruptException:
        pass
