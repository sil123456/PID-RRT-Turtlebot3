#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class SwimtoGoal(Node):

    def __init__(self):
        super().__init__('swim_to_goal')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = Pose()
        self.tolerance = 0.5
        
        self.move_to_goal()


    def pose_callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


    def get_distance(self, goal_x, goal_y):
        return sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
    

    def move_to_goal(self):
        while True:
            goal_pose = Pose()
            print('\nSuggest inputs from 1-10 to avoid hitting walls')
            goal_pose.x = float(input("Set your x goal: "))
            goal_pose.y = float(input("Set your y goal: "))
            
            vel = Twist()

            while self.get_distance(goal_pose.x, goal_pose.y) >= self.tolerance:
                # Linear velocity in the x-axis
                vel.linear.x = 1.5 * self.get_distance(goal_pose.x, goal_pose.y)
                vel.linear.y = 0.0
                vel.linear.z = 0.0

                # Angular velocity in the z-axis
                vel.angular.x = 0.0
                vel.angular.y = 0.0
                vel.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
                # vel_msg.angular.z = 4 * (self.pose.theta - atan2(self.pose.x - goal_pose.x, self.pose.y - goal_pose.y))

                self.velocity_publisher.publish(vel)
                rclpy.spin_once(self, timeout_sec=0.1)

            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.velocity_publisher.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    swim_to_goal = SwimtoGoal()

    try:
        rclpy.spin(swim_to_goal)
        # swim_to_goal.move_to_goal()  
        # while True:
        #     swim_to_goal.move_to_goal()  
    except KeyboardInterrupt:  
        pass  
    finally:
        swim_to_goal.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()