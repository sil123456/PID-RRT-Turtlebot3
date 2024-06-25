import rclpy
from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid
# from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
# import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
import cv2
from .rrt import find_path_RRT


class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')

        self.x_start_real = 0.0
        self.y_start_real = 0.0
        self.x_goal_real = 0.0
        self.y_goal_real = 0.0

        self.height = 600 # this is a random height value
        self.width = 1000 # this is a random width value
        self.resolution = 0.03 # this is a random resolution value
        self.current_map = np.zeros((self.height, self.width))

        self.point_received = False

        self.pos_subscription = self.create_subscription(Float64MultiArray, '/start_goal', self.pos_received, 10)
        self.pos_subscription  # prevent unused variable warning
        self.publisher_path = self.create_publisher(Float64MultiArray, '/trajectory', 10)
        self.recevied_param()


    def pos_received(self, msg):
        pos_info = msg.data
        self.x_start_real = pos_info[0]
        self.y_start_real = pos_info[1]
        self.x_goal_real = pos_info[2]
        self.y_goal_real = pos_info[3]
        self.point_received = True


    def recevied_param(self):
        while True:
            while (rclpy.ok() and not self.point_received):
                rclpy.spin_once(self)
            self.calculation()
            self.point_received = False


    def map_img(self, arr):
        disp_map = np.ones((self.height, self.width)) * 255
        for i in range(arr.shape[0]):
            for j in range(arr.shape[1]):
                if arr[i][j] == -1:
                    disp_map[i][j] = 100
                if arr[i][j] == 100:
                    disp_map[i][j] = 0
        im = np.array(disp_map, dtype = np.uint8)
        return im[::-1]


    def get_index_from_coordinates(self, x, y):
        x_index = int(round(x / self.resolution))
        y_index = int(round(y / self.resolution))
        return x_index, y_index
    

    def get_coordinates_from_index(self, x, y):
        x_coordinates = round(x * self.resolution, 5)
        y_coordinates = round(y * self.resolution, 5)
        return x_coordinates, y_coordinates
    
    
    def calculation(self):
        x_start_index, y_start_index = self.get_index_from_coordinates(self.x_start_real, self.y_start_real)
        x_goal_index, y_goal_index = self.get_index_from_coordinates(self.x_goal_real, self.y_goal_real)

        start, goal = ([x_start_index, y_start_index] , [x_goal_index, y_goal_index])
        
        my_map = cv2.cvtColor(self.map_img(self.current_map), cv2.COLOR_GRAY2BGR)[::-1]

        path, _ = find_path_RRT(start, goal, my_map)

        np_array = np.array(path)
        flat_path = []
        for (x, y) in np_array:
            x_coordinates, y_coordinates = self.get_coordinates_from_index(x, y)
            flat_path.append(x_coordinates)
            flat_path.append(y_coordinates)

        msg = Float64MultiArray()
        msg.data = flat_path
        self.publisher_path.publish(msg) 


def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    try:
        rclpy.spin(rrt_node)
    except KeyboardInterrupt:
        pass
    finally:
        rrt_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


