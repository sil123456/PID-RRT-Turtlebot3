import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import sin, cos, pi, sqrt
import random


class SwimNode(Node):

    def __init__(self):

        super().__init__('swim_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

        self.W = random.uniform(1, 4)
        self.H = random.uniform(1, 4)
        self.T = random.uniform(10, 15)

        # please do some trial and error to adjust the random range of values 
        # depending on your configuration environment and running speed if needed.
        # Do not set the range too large, otherwise the turtle will hit the wall.


    def timer_callback(self):

        vel = Twist()
        t = self.i
        
        dx = ((self.W * pi) / self.T) * cos((2 * pi * t)/self.T)
        dy = ((2 * self.H *pi) / self.T) * cos((4 * pi * t)/self.T)
        ddx = -((2 * self.W * pi**2) / self.T**2) * sin(2 * pi * t / self.T)
        ddy = -((8 * self.H * pi**2) / self.T**2) * sin((4 * pi * t) / self.T)

        vel.linear.x = sqrt(dx**2 + dy**2)
        vel.angular.z = (ddy * dx - dy * ddx) / (dx**2 + dy**2)
        self.publisher_.publish(vel)

        self.get_logger().info('Width: %.2f Height: %.2f Period %.2f' % (self.W, self.H, self.T))
        self.get_logger().info('Publishing: linear_x: %.5f angular_z: %.5f' % (vel.linear.x, vel.angular.z))

        if t >= self.T:
            self.i = 0
        else:
            self.i += 0.01


def main(args=None):

    rclpy.init(args=args)
    swim_node = SwimNode()
    rclpy.spin(swim_node)
    swim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
