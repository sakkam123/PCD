#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.map1_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map1_callback, 10)  # Robot1's map
        self.map2_sub = self.create_subscription(
            OccupancyGrid, '/robot2/map', self.map2_callback, 10)  # Robot2's local map
            
        # Publisher - merged map goes to /map
        self.merged_pub = self.create_publisher(
            OccupancyGrid, '/map', 10)
            
        self.map1 = None
        self.map2 = None
        self.get_logger().info("Map merger initialized - will merge into /map")

    def map1_callback(self, msg):
        self.map1 = msg
        self.try_merge_maps()

    def map2_callback(self, msg):
        self.map2 = msg
        self.try_merge_maps()

    def try_merge_maps(self):
        if self.map1 is None or self.map2 is None:
            return
            
        try:
            # Get transform from robot2's frame to robot1's frame
            transform = self.tf_buffer.lookup_transform(
                self.map1.header.frame_id,  # Target frame (robot1/map)
                self.map2.header.frame_id,  # Source frame (robot2/map)
                rclpy.time.Time())
                
            # Convert maps to numpy arrays
            map1_data = np.array(self.map1.data, dtype=np.int8).reshape(
                (self.map1.info.height, self.map1.info.width))
            map2_data = np.array(self.map2.data, dtype=np.int8).reshape(
                (self.map2.info.height, self.map2.info.width))
                
            # Create merged map (start with robot1's map)
            merged = np.copy(map1_data)
            
            # Transform and merge robot2's map
            for y in range(map2_data.shape[0]):
                for x in range(map2_data.shape[1]):
                    if map2_data[y, x] != -1:  # Only merge known cells
                        # Convert map2 cell to world coordinates
                        world_x = x * self.map2.info.resolution + self.map2.info.origin.position.x
                        world_y = y * self.map2.info.resolution + self.map2.info.origin.position.y
                        
                        # Transform to robot1's frame
                        point = tf2_geometry_msgs.PointStamped()
                        point.point.x = world_x
                        point.point.y = world_y
                        point.header = self.map2.header
                        transformed = tf2_geometry_msgs.do_transform_point(point, transform)
                        
                        # Convert to robot1's grid coordinates
                        map1_x = int((transformed.point.x - self.map1.info.origin.position.x) / self.map1.info.resolution)
                        map1_y = int((transformed.point.y - self.map1.info.origin.position.y) / self.map1.info.resolution)
                        
                        # Check bounds and merge (prioritize robot2's information)
                        if (0 <= map1_x < merged.shape[1] and 
                            0 <= map1_y < merged.shape[0]):
                            merged[map1_y, map1_x] = map2_data[y, x]
            
            # Publish merged map to /map
            merged_msg = OccupancyGrid()
            merged_msg.header.stamp = self.get_clock().now().to_msg()
            merged_msg.header.frame_id = self.map1.header.frame_id
            merged_msg.info = self.map1.info
            merged_msg.data = merged.flatten().tolist()
            
            self.merged_pub.publish(merged_msg)
            self.get_logger().debug('Published merged map to /map', throttle_duration_sec=1.0)
            
        except Exception as e:
            self.get_logger().error(f'Error merging maps: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
