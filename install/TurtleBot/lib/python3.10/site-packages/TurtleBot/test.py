import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
from scipy.spatial import KDTree
import random
import math


class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')

        self.height = 0.0
        self.width = 0.0
        self.x_origin = 0.0
        self.y_origin = 0.0
        self.resolution = 0.0
        self.data = None
        self.x_start_real = 0.0
        self.y_start_real = 0.0
        self.x_goal_real = 0.0
        self.y_goal_real = 0.0
        self.current_map = None

        self.point_received = False
        self.map_received = False


        qos_profile = self.get_qos_profile()
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.pos_subscription = self.create_subscription(Float64MultiArray, '/start_goal', self.pos_received, 10)
        self.pos_subscription  # prevent unused variable warning
        self.publisher_path = self.create_publisher(Float64MultiArray, '/trajectory', 10)
        self.recevied_param()

    def get_qos_profile(self):
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos
    

    def pos_received(self, msg):
        pos_info = msg.data
        self.x_start_real = pos_info[0]
        self.y_start_real = pos_info[1]
        self.x_goal_real = pos_info[2]
        self.y_goal_real = pos_info[3]
        self.point_received = True


    def map_callback(self, msg):
        self.height = msg.info.height
        self.width = msg.info.width
        self.x_origin = msg.info.origin.position.x
        self.y_origin = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.data = msg.data

        # Convert 1D occupancy grid data to 2D numpy array
        self.current_map = np.array(self.data).reshape((self.height, self.width))
        self.map_received = True
        # self.get_logger().info('Received map! Height: {}, Width: {}, x: {}, y: {}, resolution: {}'.format(self.height, self.width, self.x_origin, self.y_origin, self.resolution))

    def recevied_param(self):
        while (rclpy.ok() and (not self.point_received or not self.map_received)):
            rclpy.spin_once(self)
        self.calculation()


    def calculation(self):
        print(f"Map bounds in grid indices: x: [0, {self.width - 1}], y: [0, {self.height - 1}]")
        print(f"Initial cell coordinates: {self.x_start_real, self.y_start_real}")
        print(f"Final cell coordinates: {self.x_goal_real, self.y_goal_real}")

        if self.is_within_bounds(self.x_start_real, self.y_start_real) and self.is_within_bounds(self.x_goal_real, self.y_goal_real):
            start = (self.x_start_real, self.y_start_real)
            end = (self.x_goal_real, self.y_goal_real)
            path = self.rrt(start, end)
            if path:
                msg = Float64MultiArray()
                np_array = np.array(path)
                flat_path = np_array.flatten().tolist()
                msg = Float64MultiArray()
                msg.data = flat_path
                self.publisher_path.publish(msg) 
                self.plot_path(path)

            else:
                print("No path found.")
        else:
            print("Start or goal position out of map bounds.")


    def is_within_bounds(self, x, y):
        return 0 <= x < self.current_map.shape[1] and 0 <= y < self.current_map.shape[0]


    def is_free(self, point):
        x, y = point
        if self.is_within_bounds(x, y):
            return self.current_map[y, x] == 0  # Free space
        return False


    def rrt(self, start, goal, max_iter=5000, step_size=0.5):
        nodes = [start]
        parents = {start: None}
        kdtree = KDTree([start])

        for _ in range(max_iter):
            # rand_point = (random.randint(0, self.current_map.shape[1] - 1),
            #               random.randint(0, self.current_map.shape[0] - 1))
            
            rand_point = (random.randint(0, 9),
                          random.randint(0, 9))

            nearest_dist, nearest_index = kdtree.query(rand_point)
            nearest_point = nodes[nearest_index]

            direction = np.array(rand_point) - np.array(nearest_point)
            # norm = np.linalg.norm(direction)
            # if norm == 0:
            #     continue  # Avoid division by zero
            # direction = direction / norm  # Normalize the direction vector

            length = min(step_size, nearest_dist)
            # length = step_size

            new_point = np.array(nearest_point) + direction * length
            # new_point = (round(new_point[0]), round(new_point[1]))  
            new_point = tuple(map(int, new_point))
            print(new_point)

            if self.is_free(new_point):
                nodes.append(new_point)
                parents[new_point] = nearest_point
                kdtree = KDTree(nodes)

                if math.hypot(new_point[0] - goal[0], new_point[1] - goal[1]) < step_size:
                    parents[goal] = new_point
                    return self.reconstruct_path(parents, start, goal)

        return []


    def reconstruct_path(self, parents, start, goal):
        path = [goal]
        while path[-1] != start:
            path.append(parents[path[-1]])
        path.reverse()
        return path

    def plot_path(self, path):
        plt.imshow(self.current_map, cmap='binary', origin='lower')
        if path:
            path = np.array(path)
            plt.plot(path[:, 0], path[:, 1], 'r-')
            plt.plot(path[0, 0], path[0, 1], 'go')  # Start point
            plt.plot(path[-1, 0], path[-1, 1], 'bo')  # Goal point
        plt.title('RRT Path Planning')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.show()



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


