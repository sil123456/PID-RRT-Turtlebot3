import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 

 
class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        self.x_start_real = 0.0
        self.y_start_real = 0.0
        self.x_goal_real = 0.0
        self.y_goal_real = 0.0
        self.path = None

        self.publisher_point = self.create_publisher(Float64MultiArray, '/start_goal', 10)
        self.trajectory_sub = self.create_subscription(Float64MultiArray, '/trajectory', self.path_callback, 10)
        self.input_points()


    def path_callback(self, msg):
        path_info = msg.data
        path_list = [path_info[i:i+2] for i in range(0, len(path_info), 2)]
        # print("\nMotion_Planner node received the trajectory path: ", path_list)
        for a, b in path_list:
            print(a, b)

   
    def input_points(self):
        msg = Float64MultiArray() 
        self.x_start_real = float(input("Enter the x_start_real: "))
        self.y_start_real = float(input("Enter the y_start_real: "))
        self.x_goal_real = float(input("Enter the x_goal_real: "))
        self.y_goal_real = float(input("Enter the y_goal_real: "))
        msg.data = [self.x_start_real, self.y_start_real, self.x_goal_real, self.y_goal_real] 
        self.publisher_point.publish(msg) 


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