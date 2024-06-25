#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import curses


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def send_twist(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.publisher_.publish(twist)


def main():

    rclpy.init(args=None)
    node = TeleopNode()
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    try:
        while True:
            key = stdscr.getch()
            if key == ord('w'):
                node.send_twist(linear_vel=1.0, angular_vel=0.0)
                stdscr.addstr(0, 0, 'Move forward            ')
            elif key == ord('s'):
                node.send_twist(linear_vel=-1.0, angular_vel=0.0)
                stdscr.addstr(0, 0, 'Move backward           ')
            elif key == ord('a'):
                node.send_twist(linear_vel=0.0, angular_vel=1.0)
                stdscr.addstr(0, 0, 'Rotate to the left      ')
            elif key == ord('d'):
                node.send_twist(linear_vel=0.0, angular_vel=-1.0)
                stdscr.addstr(0, 0, 'Rotate to the right     ')
            elif key == -1:
                # No input - stop the turtle
                node.send_twist(linear_vel=0.0, angular_vel=0.0)

            elif key == ord('q'):
                break

            rclpy.spin_once(node, timeout_sec=0.1)  # to handle callbacks
            # rclpy.spin(node)  
            stdscr.refresh()  # Refresh the screen

    finally:
        rclpy.shutdown()
        node.destroy_node()
        stdscr.keypad(False)
        curses.nocbreak()
        curses.echo()
        curses.endwin()


if __name__ == '__main__':
    main()



