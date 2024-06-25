import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 
from nav_msgs.msg import Odometry
from math import pi, atan2, sqrt


 
class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        self.x_start = 0.0
        self.y_start = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.path_list = []

        self.rrt_pub = self.create_publisher(Float64MultiArray, '/start_goal', 10)
        self.pid_pub = self.create_publisher(Float64MultiArray, '/reference_pose', 10)

        self.trajectory_sub = self.create_subscription(Float64MultiArray, '/trajectory', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

        self.received_pose = False
        self.received_path = False

        self.input_points()


    def pose_callback(self, data):
        self.current_x  = data.pose.pose.position.x
        self.current_y  = data.pose.pose.position.y

        # convert a Quaternion into Euler angles
        x, y, z, w = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        self.current_theta = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        self.received_pose = True
        

    def path_callback(self, msg):
        path_info = msg.data
        self.path_list = [path_info[i:i+2] for i in range(0, len(path_info), 2)]
        print("\nMotion_Planner node received the trajectory path: ", self.path_list)
        self.received_path = True


    def input_points(self):
        while True:
            self.x_goal = float(input("\nEnter the x_goal (must be positive, recommend plus or minus 1.5 from current x): "))
            self.y_goal = float(input("Enter the y_goal (must be positive, recommend plus or minus 1.5 from current y): "))
            self.theta_goal = float(input("Enter the theta (-180 to 180) of the goal: "))
            self.theta_goal = self.theta_goal * pi / 180
            self.mode = float(input("Enter the mode (0 or 1): "))
            self.whole_process()


    def whole_process(self):
        while (rclpy.ok() and not self.received_pose):
            rclpy.spin_once(self)
        self.received_pose = False
        self.rrt_step()

        while (rclpy.ok() and not self.received_path):
            rclpy.spin_once(self)
        self.received_path = False
        self.pid_step()
        

    def rrt_step(self):
        msg = Float64MultiArray() 
        self.x_start = max(0.0, self.current_x) # avoid edge case
        self.y_start = max(0.0, self.current_y) # avoid edge case
        
        msg.data = [self.x_start, self.y_start, self.x_goal, self.y_goal] 
        self.rrt_pub.publish(msg) 


    def pid_step(self):
        length = len(self.path_list)
        for count, (path_x, path_y) in enumerate(self.path_list):
            rclpy.spin_once(self)
            msg = Float64MultiArray() 
            msg.data = [path_x, path_y, self.theta_goal, self.mode] 
            self.pid_pub.publish(msg)
            while rclpy.ok():
                rclpy.spin_once(self) 

                # check if the robot reach the goal for both position and angle
                if abs(sqrt((path_x - self.current_x)**2 + (path_y - self.current_y)**2)) <= 0.1 and abs(self.theta_goal - self.current_theta) <= 0.1:
                    if count == length - 1:
                        print('Reached the goal ({}, {})!'.format(self.x_goal, self.y_goal))
                    else:
                        print('Reached an intermediate point ({}, {}) on the path!'.format(path_x, path_y))
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