import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        # Subscribers for maps from robot1 and robot2
        self.map1_sub = self.create_subscription(OccupancyGrid, '/map', self.map1_callback, 10)
        self.map2_sub = self.create_subscription(OccupancyGrid, '/robot2/map', self.map2_callback, 10)

        # Publisher for the merged map
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.map1 = None
        self.map2 = None

    def map1_callback(self, msg):
        self.map1 = msg
        self.try_merge_maps()

    def map2_callback(self, msg):
        self.map2 = msg
        self.try_merge_maps()

    def try_merge_maps(self):
        if self.map1 is None or self.map2 is None:
            return  # Wait until both maps are received

        # Convert maps to NumPy arrays
        map1_data = np.array(self.map1.data).reshape((self.map1.info.height, self.map1.info.width))
        map2_data = np.array(self.map2.data).reshape((self.map2.info.height, self.map2.info.width))

        # Transformation: Assume robot2 is at (1, 0) relative to robot1
        dx, dy = int(1.0 / self.map1.info.resolution), int(0.0 / self.map1.info.resolution)
        
        # Create merged map initialized with unknown (-1)
        merged_map = np.full_like(map1_data, -1)

        # Merge map1
        for y in range(map1_data.shape[0]):
            for x in range(map1_data.shape[1]):
                if map1_data[y, x] != -1:  # If known, use map1's value
                    merged_map[y, x] = map1_data[y, x]

        # Merge map2 into the merged map
        for y in range(map2_data.shape[0]):
            for x in range(map2_data.shape[1]):
                new_x, new_y = x + dx, y + dy
                if 0 <= new_x < merged_map.shape[1] and 0 <= new_y < merged_map.shape[0]:
                    if map2_data[y, x] != -1:  # If known, use map2's value
                        merged_map[new_y, new_x] = map2_data[y, x]

        # Create and publish the new merged map
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = "map"  # Keep the same frame as robot1's map
        merged_msg.info = self.map1.info  # Use the first map's metadata (resolution, origin, etc.)
        merged_msg.data = list(merged_map.flatten())

        self.map_pub.publish(merged_msg)
        self.get_logger().info("Published the merged map!")

def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

