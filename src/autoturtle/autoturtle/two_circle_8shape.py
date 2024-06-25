#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random

class SwimNode(Node):

    def __init__(self):
        super().__init__('swim_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.curr_pose = Pose()
        self.start_pos = False
        self.start_x = 0
        self.start_y = 0

        self.prev = self.get_clock().now().seconds_nanoseconds()[0]
        self.time_switch = 0

        self.i = 0
        self.state = 0

        self.dist_threshold = 0.05
        self.time_threshold = 1.5
        # Through some trial and error, distance and time threshold are set. 

        self.random_linear_x = random.uniform(0.5, 1.2) 
        self.random_angular_z = random.uniform(0.5, 1.2)
        # Through some trial and error, range of random linear and angular velocity are set between 0.5-1.2 for VM. 
        # It could be 1.0-4.0 for Windows -> remote ssh.
        # Do not set the range too large, otherwise the turtle will hit the wall.
        # Please note that timer_period, self.dist_threshold, self.time_threshold, 
        # self.random_linear_x and self.time_threshold all five variables are correlated. So please do some trial and error
        # to adjust the values depending on the configuration environment and running speed if needed.


    def pose_callback(self, msg):
        if self.start_pos == False:
            self.start_x = msg.x
            self.start_y = msg.y
            self.start_pos = True
        else:
            self.curr_pose = msg


    def timer_callback(self):
        vel = Twist()
        self.get_logger().info('Turtle first position x: {:.5f}, y: {:.5f}'.format(self.start_x, self.start_y))
        self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.get_logger().info(f'Turtle current position x: {self.curr_pose.x:.5f}, y: {self.curr_pose.y:.5f}')

        t = self.i
        if t != 0 and (abs(self.curr_pose.x - self.start_x) < self.dist_threshold) and (abs(self.curr_pose.y - self.start_y) < self.dist_threshold):
            self.time_switch = self.get_clock().now().seconds_nanoseconds()[0]
            if self.time_switch - self.prev > self.time_threshold: 
            # this threshold is used to avoid sudden state switching. It is safe to set between 1-2 on VM.
            # (10-20 on Windows -> remote ssh).
                self.prev = self.time_switch
                self.state = 1 - self.state

        
        if self.state == 1:
            vel.linear.x = self.random_linear_x
            vel.angular.z = self.random_angular_z
            self.publisher_.publish(vel)
            self.get_logger().info('Publishing: linear_x: %.5f angular_z: %.5f' % (vel.linear.x, vel.angular.z))
        else:
            vel.linear.x = self.random_linear_x
            vel.angular.z = -self.random_angular_z
            self.publisher_.publish(vel)
            self.get_logger().info('Publishing: linear_x: %.5f angular_z: %.5f' % (vel.linear.x, vel.angular.z))

        self.get_logger().info('Current time t: {:.1f}'.format(t))
        self.i += 0.05


def main(args=None):
    rclpy.init(args=args)
    swim_node = SwimNode()
    rclpy.spin(swim_node)
    swim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
