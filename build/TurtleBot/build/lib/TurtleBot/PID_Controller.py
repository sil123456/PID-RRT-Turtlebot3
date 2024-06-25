import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from math import atan2, sqrt, pi


class SwimToGoal(Node):
    def __init__(self):
        super().__init__('swim_to_goal')

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        self.mode = 0.0
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

        # mode 0 PID parameter
        self.Kp_pos_0 = 0.2 #1.5
        self.Ki_pos_0 = 0.00001 #1.5
        self.Kd_pos_0 = 0.01 #1.5
        self.Kp_ang_0 = 0.2 #1.5
        self.Ki_ang_0 = 0.00001 #1.5
        self.Kd_ang_0 = 0.01 #1.5

        # mode 1 PID parameter
        self.Kp_pos_1 = 0.2 #1.5
        self.Ki_pos_1 = 0.00000 #1.5
        self.Kd_pos_1 = 0.01 #1.5
        self.Kp_ang_1 = 0.2 #1.5
        self.Ki_ang_1 = 0.00000 #1.5
        self.Kd_ang_1 = 0.001 #1.5

        # flag to check whether PID controller have received goal position
        self.received = False

        self.turtle_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.subscription_1 = self.create_subscription(Float64MultiArray, '/reference_pose', self.pos_received, 10)
        self.subscription_1  # prevent unused variable warning
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_to_goal()

    # extract current x, y and theta
    def pose_callback(self, data):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        # convert a Quaternion into Euler angles
        x, y, z, w = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        self.current_theta = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        #self.current_theta = data.pose.pose.orientation.w * pi

        #self.get_logger().info('current_x: %.5f current_y: %.5f current_theta: %.5f' % (self.current_x, self.current_y, self.current_theta))
        print('current_x: {:.5f} current_y: {:.5f} current_theta: {:.5f}'.format(self.current_x, self.current_y, self.current_theta))
        print('goal_x: {:.5f} goal_y: {:.5f} goal_theta: {:.5f}'.format(self.x_goal, self.y_goal, self.theta_goal))


    # extract goal x, y, theta and mode choice from the Motion_Planner, then execute PID control
    def pos_received(self, msg):
        robot_info = msg.data
        self.x_goal = robot_info[0]
        self.y_goal = robot_info[1]
        self.theta_goal = robot_info[2]
        self.mode = robot_info[3]
        self.received = True


    # execute mode 0 or mode 1 PID control
    def move_to_goal(self):
        while True:
            while (rclpy.ok() and not self.received):
                rclpy.spin_once(self)
            # self.received = False
            if int(self.mode) == 0:
                self.mode_0()
            elif int(self.mode) == 1:
                self.mode_1()
                

    def mode_0(self):
        rclpy.spin_once(self)
        self.prev_error_position = 0.0
        self.integral_error_position = 0.0
        self.prev_error_angle1 = 0.0
        self.integral_error_angle1 = 0.0
        self.prev_error_angle2 = 0.0
        self.integral_error_angle2 = 0.0
    
        vel = Twist()

        # activate the PID for the angular velocity first until the robot faces the reference point, check the condition
        while abs(atan2((self.y_goal - self.current_y), (self.x_goal - self.current_x)) - self.current_theta) > 0.001:

            self.error_angle = atan2((self.y_goal - self.current_y), (self.x_goal - self.current_x)) - self.current_theta

            if self.error_angle < -pi:
                self.error_angle += 2 * pi
            if self.error_angle > pi:
                self.error_angle -= 2 * pi

            integral_error_angle1 = self.integral_error_angle1 + self.error_angle
            derivative_error_angle1 = self.error_angle - self.prev_error_angle1

            self.prev_error_angle1 = self.error_angle
            self.integral_error_angle1 = integral_error_angle1

            p_ang = self.Kp_ang_0 * self.error_angle
            i_ang = self.Ki_ang_0 * integral_error_angle1 
            d_ang = self.Kd_ang_0 * derivative_error_angle1
            pid_ang = p_ang + i_ang + d_ang

            # update velocity
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = pid_ang

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)

        vel.angular.z = 0.0
        self.vel_pub.publish(vel)

        # activate the linear velocity controller until the robot gets to the goal, check the condition
        while abs(sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)) > 0.098:

            self.error_position = sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)
            integral_error_position = self.integral_error_position + self.error_position
            derivative_error_position = self.error_position - self.prev_error_position

            self.prev_error_position = self.error_position
            self.integral_error_position = integral_error_position

            p_pos = self.Kp_pos_0 * self.error_position
            i_pos = self.Ki_pos_0 * integral_error_position 
            d_pos = self.Kd_pos_0 * derivative_error_position
            pid_pos = p_pos + i_pos + d_pos

            # update velocity
            vel.linear.x = pid_pos
            vel.linear.y = 0.0
            vel.linear.z = 0.0

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        vel.linear.x = 0.0
        self.vel_pub.publish(vel)

        # activate the angular velocity controller to turn the robot towards the final angle, check the condition
        while abs(self.theta_goal - self.current_theta) > 0.1:

            error_ang2 = self.theta_goal - self.current_theta

            if error_ang2 < -pi:
                error_ang2 += 2 * pi
            if error_ang2 > pi:
                error_ang2 -= 2 * pi

            integral_error_angle2 = self.integral_error_angle2 + error_ang2
            derivative_error_angle2 = error_ang2 - self.prev_error_angle2

            self.prev_error_angle2 = error_ang2
            self.integral_error_angle2 = integral_error_angle2

            p_ang = self.Kp_ang_0 * error_ang2
            i_ang = self.Ki_ang_0 * integral_error_angle2 
            d_ang = self.Kd_ang_0 * derivative_error_angle2
            pid_ang = p_ang + i_ang + d_ang

            # update velocity
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = pid_ang

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
        

        vel.angular.z = 0.0
        self.vel_pub.publish(vel)
        self.received = False
        
        
    def mode_1(self):
        rclpy.spin_once(self)
        self.prev_error_position = 0.0
        self.integral_error_position = 0.0
        self.prev_error_angle1 = 0.0
        self.integral_error_angle1 = 0.0

        vel = Twist()

        while rclpy.ok():
            self.error_position = sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)
            integral_error_position = self.integral_error_position + self.error_position
            derivative_error_position = self.error_position - self.prev_error_position

            self.error_angle = atan2((self.y_goal - self.current_y), (self.x_goal - self.current_x)) - self.current_theta
            if self.error_angle < -pi:
                self.error_angle += 2 * pi
            if self.error_angle > pi:
                self.error_angle -= 2 * pi
            if self.error_position <= 0.08:
                self.error_angle = abs(self.theta_goal - self.current_theta)


            # activate both the angular and the linear controller to reach the goal position and pose, check the condition
            if abs(self.error_position) <= 0.1 and abs(self.theta_goal - self.current_theta) <= 0.1:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                rclpy.spin_once(self, timeout_sec=0.1)
                self.received = False
                break

            # self.error_angle  = atan2(sin(self.error_angle), cos(self.error_angle))
            integral_error_angle = self.integral_error_angle1 + self.error_angle 
            derivative_error_angle = self.error_angle - self.prev_error_angle1

            self.prev_error_position = self.error_position
            self.integral_error_position = integral_error_position
            self.prev_error_angle1 = self.error_angle 
            self.integral_error_angle1 = integral_error_angle

            p_pos = self.Kp_pos_1 * self.error_position
            i_pos = self.Ki_pos_1 * integral_error_position 
            d_pos = self.Kd_pos_1 * derivative_error_position
            pid_pos = p_pos + i_pos + d_pos

            p_ang = self.Kp_ang_1 * self.error_angle 
            i_ang = self.Ki_ang_1 * integral_error_angle 
            d_ang = self.Kd_ang_1 * derivative_error_angle
            pid_ang = p_ang + i_ang + d_ang

            # update velocity
            vel.linear.x = pid_pos
            vel.linear.y = 0.0
            vel.linear.z = 0.0

            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = pid_ang

            self.vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
                    


def main(args=None):
    rclpy.init(args=args)
    swim_to_goal = SwimToGoal()
    try:
        rclpy.spin(swim_to_goal)
    except KeyboardInterrupt:
        pass
    finally:
        swim_to_goal.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()

