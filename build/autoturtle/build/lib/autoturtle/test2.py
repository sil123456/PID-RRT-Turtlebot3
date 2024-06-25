import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, sin, cos, asin, pi, atan


class SwimToGoal(Node):
    def __init__(self):
        super().__init__('swim_to_goal')
        print("Node created")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_pos', 1), #1.5
                ('Ki_pos', 0), #0.8
                ('Kd_pos', 0.01), #0.1
                ('Kp_ang', 1.5), #1.5
                ('Ki_ang', 0), #0.8
                ('Kd_ang', 0.01), #0.1
            ]
        )

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.error_position = 0.0
        self.error_angle = 0.0
        self.prev_error_position = 0.0
        self.integral_error_position = 0.0
        self.prev_error_angle = 0.0
        self.integral_error_angle = 0.0

        # self.get_parameters()
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # self.get_goal()


    def get_parameters(self):
        self.Kp_pos = self.get_parameter('Kp_pos').value
        self.Ki_pos = self.get_parameter('Ki_pos').value
        self.Kd_pos = self.get_parameter('Kd_pos').value
        self.Kp_ang = self.get_parameter('Kp_ang').value
        self.Ki_ang = self.get_parameter('Ki_ang').value
        self.Kd_ang = self.get_parameter('Kd_ang').value


    def pose_callback(self, data):
        print("Received odometry message")
        self.current_x = data.x
        self.current_y = data.y
        self.current_theta = data.theta


        self.get_logger().info('current_x: %.5f current_y: %.5f current_theta: %.5f' % (self.current_x, self.current_y, self.current_theta))



    def move_to_goal(self):


        self.x_goal = float(input("Enter the x-coordinate of the goal: "))
        self.y_goal = float(input("Enter the y-coordinate of the goal: "))
        self.theta_goal = float(input("Enter the theta (-180 to 180) of the goal: "))
        self.theta_goal = self.theta_goal * pi / 180

        vel = Twist()

        while rclpy.ok():

        # while abs(sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)) > 0.1 or abs(atan(self.theta_goal - self.current_theta)) > 0.1:

            # PID control
            self.error_position = sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)
            integral_error_position = self.integral_error_position + self.error_position
            derivative_error_position = self.error_position - self.prev_error_position

            self.error_angle = atan2((self.y_goal - self.current_y), (self.x_goal - self.current_x)) - self.current_theta



            # if self.error_angle < -pi:
            #     self.error_angle += 2 * pi
            # if self.error_angle > pi:
            #     self.error_angle -= 2 * pi


            # print(abs(self.error_position), abs(atan(self.theta_goal - self.current_theta)))

            if abs(self.error_position) <= 0.5 and abs(self.theta_goal - self.current_theta) <= 1:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                rclpy.spin_once(self, timeout_sec=0.1)
                print('REACH')
                break



            # self.error_angle  = atan2(sin(self.error_angle), cos(self.error_angle))
            integral_error_angle = self.integral_error_angle + self.error_angle 
            derivative_error_angle = self.error_angle - self.prev_error_angle


            ## ADD YOUR CODE

            self.prev_error_position = self.error_position
            self.integral_error_position = integral_error_position
            self.prev_error_angle = self.error_angle 
            self.integral_error_angle = integral_error_angle


            p_pos = self.Kp_pos * self.error_position
            i_pos = self.Ki_pos * integral_error_position 
            d_pos = self.Kd_pos * derivative_error_position
            pid_pos = p_pos + i_pos + d_pos


            p_ang = self.Kp_ang * self.error_angle 
            i_ang = self.Ki_ang * integral_error_angle 
            d_ang = self.Kd_ang * derivative_error_angle
            pid_ang = p_ang + i_ang + d_ang


            if (pid_pos < -0.7):
                pid_pos = -0.7
            elif (pid_pos > 0.7):
                pid_pos = 0.7


            if (pid_ang < -pi):
                pid_ang = -pi
            elif (pid_ang > pi):
                pid_ang = pi

            vel.linear.x = pid_pos
            vel.angular.z = pid_ang



            # vel.linear.x = pid_pos
            # vel.linear.y = 0.0
            # vel.linear.z = 0.0

            # vel.angular.x = 0.0
            # vel.angular.y = 0.0
            # vel.angular.z = pid_ang

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)



def main(args=None):
    rclpy.init(args=args)
    swim_to_goal = SwimToGoal()

    try:
        swim_to_goal.get_parameters()
        swim_to_goal.move_to_goal()  
    except KeyboardInterrupt:  
        pass  
    finally:
        swim_to_goal.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
