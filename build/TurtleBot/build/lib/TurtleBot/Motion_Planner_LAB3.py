# from .PID_Controller import SwimToGoal  #for VM
# from .PID_Controller import SwimToGoal # for Ubuntu
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 
from nav_msgs.msg import Odometry
from math import pi, atan2, sqrt

 
class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        self.mode = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.publisher_param = self.create_publisher(Float64MultiArray, '/reference_pose', 10)
        self.turtle_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.check_goal()


    def pose_callback(self, data):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        # convert a Quaternion into Euler angles
        x, y, z, w = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        self.current_theta = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        #self.current_theta = data.pose.pose.orientation.w * pi

   
    # ask for user inputs for goal and mode choice, and check if the robot reaches the final goal
    def check_goal(self):
        while True:
            msg = Float64MultiArray() 
            self.x_goal = float(input("Enter the x-coordinate of the goal: "))
            self.y_goal = float(input("Enter the y-coordinate of the goal: "))
            self.theta_goal = float(input("Enter the theta (-180 to 180) of the goal: "))
            self.theta_goal = self.theta_goal * pi / 180
            self.mode = float(input("Enter the mode (0 or 1): "))

            msg.data = [self.x_goal, self.y_goal, self.theta_goal, self.mode] 
            self.publisher_param.publish(msg) 
            while rclpy.ok():
                rclpy.spin_once(self) 
                 
                # check if the robot reach the final goal for both position and angle
                if abs(sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)) <= 0.1 and abs(self.theta_goal - self.current_theta) <= 0.1:
                    print('Reached the goal!\n')
                    break

            

def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanner()
    try:
        rclpy.spin(motion_planner)
    except KeyboardInterrupt:  
        pass  
    finally:
        motion_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()