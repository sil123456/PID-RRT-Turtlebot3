import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos, asin, pi


class SwimToGoal(Node):
    def __init__(self):
        super().__init__('swim_to_goal')
        print("Node created")
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('Kp_pos', 0.1), #1.5
        #         ('Ki_pos', 0.001), #0.8
        #         ('Kd_pos', 0.01), #0.1
        #         ('Kp_ang', 0.1), #1.5
        #         ('Ki_ang', 0.001), #0.8
        #         ('Kd_ang', 0.01), #0.1
        #     ]
        # )

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
        self.prev_error_angle1 = 0.0
        self.integral_error_angle1 = 0.0
        self.prev_error_angle2 = 0.0
        self.integral_error_angle2 = 0.0


        self.Kp_pos_0 = 0.1 #1.5
        self.Ki_pos_0 = 0.00001 #1.5
        self.Kd_pos_0 = 0.01 #1.5
        self.Kp_ang_0 = 0.1 #1.5
        self.Ki_ang_0 = 0.00001 #1.5
        self.Kd_ang_0 = 0.01 #1.5


        self.Kp_pos_1 = 0.1 #1.5
        self.Ki_pos_1 = 0.001 #1.5
        self.Kd_pos_1 = 0.01 #1.5
        self.Kp_ang_1 = 0.1 #1.5
        self.Ki_ang_1 = 0.001 #1.5
        self.Kd_ang_1 = 0.01 #1.5


        # self.get_parameters()
        self.turtle_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.get_goal()


        self.move_to_goal()


    # def get_parameters(self):
    #     self.Kp_pos = self.get_parameter('Kp_pos').value
    #     self.Ki_pos = self.get_parameter('Ki_pos').value
    #     self.Kd_pos = self.get_parameter('Kd_pos').value
    #     self.Kp_ang = self.get_parameter('Kp_ang').value
    #     self.Ki_ang = self.get_parameter('Ki_ang').value
    #     self.Kd_ang = self.get_parameter('Kd_ang').value


    def pose_callback(self, data):
        print("Received odometry message")
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        x, y, z, w = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        self.current_theta = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        #self.current_theta = data.pose.pose.orientation.w * pi
        # self.current_theta = data.pose.pose.orientation.z * 2.19 #2.19



        self.get_logger().info('current_x: %.5f current_y: %.5f current_theta: %.5f' % (self.current_x, self.current_y, self.current_theta))





    # def get_goal(self):
    #     self.x_goal = float(input("Enter the x-coordinate of the goal: "))
    #     self.y_goal = float(input("Enter the y-coordinate of the goal: "))
    #     self.theta_goal = float(input("Enter the theta (-180 to 180) of the goal: "))
    #     self.theta_goal = self.theta_goal * pi / 180
    #     self.move_to_goal()


    def move_to_goal(self):


        self.x_goal = float(input("Enter the x-coordinate of the goal: "))
        self.y_goal = float(input("Enter the y-coordinate of the goal: "))
        self.theta_goal = float(input("Enter the theta (-180 to 180) of the goal: "))
        self.theta_goal = self.theta_goal * pi / 180

        vel = Twist()

        while abs(atan2((self.y_goal - self.current_y), (self.x_goal - self.current_x)) - self.current_theta) > 0.005:

            # PID control

            self.error_angle = atan2((self.y_goal - self.current_y), (self.x_goal - self.current_x)) - self.current_theta


            if self.error_angle < -pi:
                self.error_angle += 2 * pi
            if self.error_angle > pi:
                self.error_angle -= 2 * pi

            integral_error_angle1 = self.integral_error_angle1 + self.error_angle
            derivative_error_angle1 = self.error_angle - self.prev_error_angle1

            ## ADD YOUR CODE

            self.prev_error_angle1 = self.error_angle
            self.integral_error_angle1 = integral_error_angle1


            p_ang = self.Kp_ang_0 * self.error_angle
            i_ang = self.Ki_ang_0 * integral_error_angle1 
            d_ang = self.Kd_ang_0 * derivative_error_angle1
            pid_ang = p_ang + i_ang + d_ang



            # if (pid_ang < -pi):
            #     pid_ang = -pi
            # elif (pid_ang > pi):
            #     pid_ang = pi

            # Angular velocity in the z-axis
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = pid_ang

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)

        vel.angular.z = 0.0
        self.vel_pub.publish(vel)
        


        while abs(sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)) > 0.1:

            # PID control
            self.error_position = sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)
            integral_error_position = self.integral_error_position + self.error_position
            derivative_error_position = self.error_position - self.prev_error_position


            ## ADD YOUR CODE

            self.prev_error_position = self.error_position
            self.integral_error_position = integral_error_position


            p_pos = self.Kp_pos_0 * self.error_position
            i_pos = self.Ki_pos_0 * integral_error_position 
            d_pos = self.Kd_pos_0 * derivative_error_position
            pid_pos = p_pos + i_pos + d_pos
            

            # if (pid_pos < -0.7):
            #     pid_pos = -0.7
            # elif (pid_pos > 0.7):
            #     pid_pos = 0.7



            vel.linear.x = pid_pos
            vel.linear.y = 0.0
            vel.linear.z = 0.0

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        vel.linear.x = 0.0
        self.vel_pub.publish(vel)



        while abs(self.theta_goal - self.current_theta) > 0.01:

            # PID control
            error_ang2 = self.theta_goal - self.current_theta


            if error_ang2 < -pi:
                error_ang2 += 2 * pi
            if error_ang2 > pi:
                error_ang2 -= 2 * pi

            integral_error_angle2 = self.integral_error_angle2 + error_ang2
            derivative_error_angle2 = error_ang2 - self.prev_error_angle2

            ## ADD YOUR CODE

            self.prev_error_angle2 = error_ang2
            self.integral_error_angle2 = integral_error_angle2


            p_ang = self.Kp_ang_0 * error_ang2
            i_ang = self.Ki_ang_0 * integral_error_angle2 
            d_ang = self.Kd_ang_0 * derivative_error_angle2
            pid_ang = p_ang + i_ang + d_ang




            # if (pid_ang < -pi):
            #     pid_ang = -pi
            # elif (pid_ang > pi):
            #     pid_ang = pi
            # Angular velocity in the z-axis
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = pid_ang

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
        

        vel.angular.z = 0.0
        self.vel_pub.publish(vel)



# def main(args=None):
#     rclpy.init(args=args)
#     swim_to_goal = SwimToGoal()

#     try:
#         swim_to_goal.get_parameters()
#         swim_to_goal.move_to_goal()  
#     except KeyboardInterrupt:  
#         pass  
#     finally:
#         swim_to_goal.destroy_node()
#         rclpy.shutdown()



# if __name__ == '__main__':
#     main()
