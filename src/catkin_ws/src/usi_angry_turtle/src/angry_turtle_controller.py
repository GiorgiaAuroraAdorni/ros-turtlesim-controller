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
    PEN_ON = SetPenRequest(255, 255, 255, 3, 0)
    PEN_OFF = SetPenRequest(0, 0, 0, 0, 1)

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
        """
        Euclidean distance between current pose and the goal.
        :param goal_pose:
        :return:
        """
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=4):
        """

        :param goal_pose:
        :param constant:
        :return:
        """
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """

        :param goal_pose:
        :return:
        """
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=12):
        """

        :param goal_pose:
        :param constant:
        :return:
        """
        # return constant * (self.steering_angle(goal_pose) - self.pose.theta)
        return constant * atan2(sin(self.steering_angle(goal_pose) - self.pose.theta),
                                cos(self.steering_angle(goal_pose) - self.pose.theta))

    def angle_difference(self, goal_pose):
        return atan2(sin(goal_pose.theta - self.pose.theta), cos(goal_pose.theta - self.pose.theta))

    def angular_vel_rot(self, goal_pose, constant=12):
        """

        :param goal_pose:
        :param constant:
        :return:
        """
        return constant * self.angle_difference(goal_pose)

    def stop_walking(self):
        # Stopping our robot after the movement is over.
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def go_to_initial_position(self, tollerance=0.1):
        """After finding a turtle, return to the initial point and restart writing"""
        print 'Positioning'
        self.srv_setpen(self.PEN_OFF)

        init_pose = Pose(x=1, y=8, theta=3 * pi / 2)
        while self.euclidean_distance(init_pose) >= tollerance:
            self.move_to_goal(init_pose)

        self.rotate(init_pose)

        print 'Ready'

        self.writing(tollerance)

    def rotate(self, rotation_pose, rot_tollerance=0.017):
        while abs(self.angle_difference(rotation_pose)) >= rot_tollerance:
            self.rotate_to_goal(rotation_pose)

        # Forcing our robot to stop
        self.stop_walking()

    def writing(self, tollerance, pursuing_tolerance=2, p=None, pen_offline=None):
        """

        :param tollerance:
        :param pursuing_tolerance:
        :param p:
        :param pen_offline:
        """
        if pen_offline is None:
            pen_offline = [4, 8]

        if p is None:
            p = [Pose(x=1, y=5, theta=11 * pi / 6), Pose(x=2, y=4, theta=pi / 6), Pose(x=3, y=5, theta=pi / 2),
                 Pose(x=3, y=8, theta=0), Pose(x=7, y=8, theta=7 * pi / 6), Pose(x=4, y=6.67, theta=11 * pi / 6),
                 Pose(x=7, y=5.34, theta=11 * pi / 6), Pose(x=4, y=4, theta=7 * pi / 6), Pose(x=9, y=4, theta=0),
                 Pose(x=9, y=8, theta=pi / 2)]

        write = True
        self.srv_setpen(self.PEN_ON)

        for idx, goal_pose in enumerate(p):
            if idx in pen_offline:
                self.srv_setpen(self.PEN_OFF)

            while self.euclidean_distance(goal_pose) >= tollerance:
                self.move_to_goal(goal_pose)

                # See how far are the turtles
                # TODO insegui la tartaruga piu vicina
                if self.euclidean_distance(self.pose_t1) < pursuing_tolerance:
                    self.srv_setpen(self.PEN_OFF)
                    print 'Found another turtle'
                    write = False
                    break

            if not write:
                break

            self.rotate(goal_pose)
            self.srv_setpen(self.PEN_ON)

        # FIXME culo
        if not write:
            self.become_angry()

    def become_angry(self):
        """
        Move the hunter to the closer turtle.
        The angry turtle tries to look ahead m meter in front of the offender to intercept it.
        m is directly proportional to the current speed (of the target turtle) times the current distance
        """
        print 'Pursuing the offender turtle'
        self.srv_setpen(self.PEN_OFF)

        constant = 3
        m = constant * self.pose_t1.linear_velocity * self.euclidean_distance(self.pose_t1)
        print 'linear velocity'
        print self.pose_t1.linear_velocity
        print 'self.euclidean_distance(self.pose_t1)'
        print self.euclidean_distance(self.pose_t1)
        print 'm'
        print m

        goal_pose = Pose()
        goal_pose.x = self.pose_t1.x + m * cos(self.pose_t1.theta)
        goal_pose.y = self.pose_t1.y + m * sin(self.pose_t1.theta)

        while self.euclidean_distance(self.pose_t1) >= 0.1:
            self.move_to_goal(goal_pose)

        # After finding a turtle, reset the stage
        print 'Killing turtle target'
        try:
            self.kill_turtle("turtleTarget")
            print 'Killed turtle enemy'
        except rospy.ServiceException:
            pass
        self.clear()

        print 'returning'
        self.go_to_initial_position()

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
