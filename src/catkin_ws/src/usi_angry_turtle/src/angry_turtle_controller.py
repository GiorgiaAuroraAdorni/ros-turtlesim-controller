#!/usr/bin/env python
import random
from math import *
from threading import Thread

import rospy
from geometry_msgs.msg import *
from std_srvs.srv import *
from turtlesim.msg import *
from turtlesim.srv import *


class TurtleState(enumerate):
    WRITING_STATE = "writing"
    ANGRY_STATE = "angry"
    POSITIONING_STATE = "positioning"
    FINAL_STATE = "final"


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

        self.state = None
        self.total_turtles = 4
        self.turtles_alive = self.total_turtles
        self.goal_turtle_name = None

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
        :param name:
        """
        data.x = round(data.x, 4)
        data.y = round(data.y, 4)
        data.linear_velocity = round(data.linear_velocity, 4)

        self.pose_target_turtles[name] = data

    def sleep(self):
        """ """
        self.rate.sleep()
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException

    def euclidean_distance(self, goal_pose):
        """
        :param goal_pose
        :return: Euclidean distance between current pose and the goal pose
        """
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=4):
        """
        :param goal_pose
        :param constant
        :return: clipped linear velocity
        """
        velocity = constant * self.euclidean_distance(goal_pose)
        return min(max(-5, velocity), 5)

    def steering_angle(self, goal_pose):
        """
        :param goal_pose:
        :return: steering angle
        """
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=8):
        """
        :param goal_pose:
        :param constant:
        :return: angular velocity
        """
        # return constant * (self.steering_angle(goal_pose) - self.pose.theta)
        return constant * atan2(sin(self.steering_angle(goal_pose) - self.pose.theta),
                                cos(self.steering_angle(goal_pose) - self.pose.theta))

    def angle_difference(self, goal_pose):
        """
        :param goal_pose:
        :return: the difference between the current angle and the goal angle
        """
        return atan2(sin(goal_pose.theta - self.pose.theta), cos(goal_pose.theta - self.pose.theta))

    def angular_vel_rot(self, goal_pose, constant=12):
        """
        :param goal_pose:
        :param constant:
        :return: the angular velocity computed using the angle difference
        """
        return constant * self.angle_difference(goal_pose)

    def stop_walking(self):
        """
        Stop our robot after the movement is over
        """
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def go_to_initial_position(self, tollerance=0.1):
        """After finding a turtle, return to the initial point and restart writing
        :param tollerance:
        """
        self.srv_setpen(self.PEN_OFF)

        init_pose = Pose(x=1, y=8, theta=3 * pi / 2)
        while self.euclidean_distance(init_pose) >= tollerance:
            self.move_to_goal(init_pose)

        self.rotate(init_pose)
        self.state = TurtleState.WRITING_STATE

    def rotate(self, rotation_pose, rot_tollerance=0.017):
        """

        :param rotation_pose:
        :param rot_tollerance:
        """
        while abs(self.angle_difference(rotation_pose)) >= rot_tollerance:
            self.rotate_to_goal(rotation_pose)

        # Forcing our robot to stop
        self.stop_walking()

    def writing(self, tollerance=0.1, pursuing_tolerance=2, p=None, pen_offline=None):
        """
        :param tollerance:
        :param pursuing_tolerance:
        :param p:
        :param pen_offline:
        """
        if pen_offline is None:
            pen_offline = [5, 11]

        if p is None:
            p = [Pose(x=1, y=5, theta=5 * pi / 3), Pose(x=2, y=4, theta=0), Pose(x=3, y=4, theta=pi / 6),
                 Pose(x=4, y=5, theta=pi / 2), Pose(x=4, y=8, theta=0),

                 Pose(x=8, y=8, theta=5 * pi / 6), Pose(x=5.5, y=7, theta=5 * pi / 4),
                 Pose(x=5.5, y=6.3, theta=11 * pi / 6), Pose(x=8, y=5.7, theta=7 * pi / 4),
                 Pose(x=8, y=5, theta=4 * pi / 3), Pose(x=5.5, y=4, theta=0),

                 Pose(x=9.5, y=4, theta=pi / 2), Pose(x=9.5, y=8, theta=pi / 2)]

        self.srv_setpen(self.PEN_ON)
        self.goal_turtle_name = None

        for idx, goal_pose in enumerate(p):
            if idx in pen_offline:
                self.srv_setpen(self.PEN_OFF)

            while self.euclidean_distance(goal_pose) >= tollerance:
                self.move_to_goal(goal_pose)
                self.goal_turtle_name = self.get_closer_turtle()

                if self.turtles_alive == 0:
                    continue

                if self.euclidean_distance(self.pose_target_turtles[self.goal_turtle_name]) < pursuing_tolerance:
                    self.srv_setpen(self.PEN_OFF)
                    print 'Turtle %s to close' % self.goal_turtle_name
                    self.state = TurtleState.ANGRY_STATE
                    break

            if self.state == TurtleState.ANGRY_STATE:
                break
            else:
                self.rotate(goal_pose)
                self.srv_setpen(self.PEN_ON)

        if not self.state == TurtleState.ANGRY_STATE:
            if self.turtles_alive > 0:
                print 'Congratulation, the turtle wrote "USI", but there are some turtles still alive.'
                self.clear()
                self.state = TurtleState.POSITIONING_STATE
            else:
                self.state = None

    def get_closer_turtle(self):
        """
         :return: the name of the closer turtle
        """
        min_distance = float("inf")
        min_turtle = None

        for name, pose in self.pose_target_turtles.items():
            distance = self.euclidean_distance(pose)

            if distance < min_distance:
                min_distance = distance
                min_turtle = name

        self.goal_turtle_name = min_turtle

        return self.goal_turtle_name

    def get_future_pose(self):
        """
        :return: the pose of the target turtle and the goal pose of the turtle "in the future"
        """
        target_pose = self.pose_target_turtles[self.goal_turtle_name]

        constant = 1.1

        m = constant * target_pose.linear_velocity * self.euclidean_distance(target_pose)

        goal_pose = Pose()
        goal_pose.x = target_pose.x + m * cos(target_pose.theta)
        goal_pose.y = target_pose.y + m * sin(target_pose.theta)

        return goal_pose, target_pose

    def become_angry(self, capture_tollerance=0.5):
        """
        Move the hunter to the closer turtle.
        The angry turtle tries to look ahead m meter in front of the offender to intercept it.
        m is directly proportional to the current speed (of the target turtle) times the current distance.
        """
        self.srv_setpen(self.PEN_OFF)

        goal_pose, target_pose = self.get_future_pose()

        while self.euclidean_distance(target_pose) >= capture_tollerance:
            self.move_to_goal(goal_pose)
            goal_pose, target_pose = self.get_future_pose()

        # After capturing the turtle kill it and reset the stage
        try:
            self.kill_turtle(self.goal_turtle_name)
            print 'Killed ' + self.goal_turtle_name

            self.turtles_alive -= 1
            del self.pose_target_turtles[self.goal_turtle_name]
            self.goal_turtle_name = None

        except rospy.ServiceException:
            pass
        self.clear()

        if self.turtles_alive > 0:
            self.state = TurtleState.POSITIONING_STATE
            print 'Returning...'
        else:
            self.state = TurtleState.FINAL_STATE

    def move_to_goal(self, goal_pose):
        """
        Moves the turtle to the goal.
        :param goal_pose:
        """
        # Proportional (P) controller
        self.vel_msg.linear.x = self.linear_vel(goal_pose)  # Linear velocity in the x-axis.
        self.vel_msg.angular.z = self.angular_vel(goal_pose)  # Angular velocity in the z-axis.

        self.velocity_publisher.publish(self.vel_msg)

    def rotate_to_goal(self, goal_pose):
        """
        Rotate the turtle to reach the orientation pose.
        :param goal_pose:
        """
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = self.angular_vel_rot(goal_pose)

        self.velocity_publisher.publish(self.vel_msg)

    def main(self):
        """
        Create a new tread that generates and moves the target turtles
        """
        thread = TargetsController(args=[self])
        thread.start()

        self.state = TurtleState.POSITIONING_STATE

        while not rospy.is_shutdown():
            if self.state == TurtleState.POSITIONING_STATE:
                print 'Positioning...'
                self.go_to_initial_position()

            elif self.state == TurtleState.WRITING_STATE:
                print 'Start to write...'
                self.writing()

            elif self.state == TurtleState.ANGRY_STATE:
                print 'Pursuing the offender turtle...'
                self.become_angry()

            elif self.state == TurtleState.FINAL_STATE:
                print 'All the turtles have been killed. Write "USI".'
                self.go_to_initial_position()
                self.writing()
                print 'Congratulation, the turtle wrote "USI" and killed all the enemy turtles.'

            elif self.state is None:
                self.stop_walking()

            self.sleep()
        # If we press control + C, the node will stop.
        rospy.spin()


class TargetsController(Thread):

    def __init__(self, args=()):
        super(TargetsController, self).__init__()
        self.args = args[0]

        self.spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        self.vel_msg = Twist()

        self.target_velocity_publisher = dict()

        self.rate = rospy.Rate(10)

        self.total_turtles = self.args.total_turtles

        return

    def spawn_turtles(self, t):
        """
        Spawn in a random position the turtle
        :param t: the actual index of the turtle
        """
        name = 'turtleTarget' + str(t)

        offender_x = random.randint(1, 12)
        offender_y = random.randint(1, 12)

        self.spawn_turtle(offender_x, offender_y, 0, name)
        rospy.Subscriber('/%s/pose' % name, Pose, self.args.turtle_target_pose, name)

        self.target_velocity_publisher[name] = rospy.Publisher('/%s/cmd_vel' % name, Twist, queue_size=10)
        rospy.wait_for_service('/%s/set_pen' % name)

        set_pen = rospy.ServiceProxy('/%s/set_pen' % name, SetPen)
        set_pen(self.args.PEN_OFF)

    def random_walking(self, t):
        """
        Lets the turtle walk randomly if it is not a teleoperated turtle
        :param t: the actual index of the turtle
        """
        name = 'turtleTarget' + str(t)

        self.vel_msg.linear.x = 5 - random.random() * 10
        self.vel_msg.angular.z = 2 - random.random() * 4
        self.target_velocity_publisher[name].publish(self.vel_msg)

    def run(self):
        """
        Function that handles the tread that has to generate and move the enemy turtles
        """
        for t in range(self.total_turtles):
            self.spawn_turtles(t)

        while not rospy.is_shutdown():
            for t in range(self.total_turtles):
                self.random_walking(t)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.main()
    except rospy.ROSInterruptException:
        pass
