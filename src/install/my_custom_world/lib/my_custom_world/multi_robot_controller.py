#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotController(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        self.robot_name = robot_name
        self.cmd_vel_topic = f'/{robot_name}/cmd_vel' if robot_name == 'robot2' else '/cmd_vel'
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(0.1, self.move_robot)  # 10Hz control loop
        self.start_time = time.time()
        self.get_logger().info(f"Controller started for {robot_name} on {self.cmd_vel_topic}")

    def move_robot(self):
        msg = Twist()
        current_time = time.time() - self.start_time

        if self.robot_name == 'burger':
            # Square pattern (forward 2s, turn 2s)
            cycle = current_time % 8
            if cycle < 2.0:
                msg.linear.x, msg.angular.z = 0.2, 0.0
            elif cycle < 4.0:
                msg.linear.x, msg.angular.z = 0.0, 0.5
            elif cycle < 6.0:
                msg.linear.x, msg.angular.z = 0.2, 0.0
            else:
                msg.linear.x, msg.angular.z = 0.0, 0.5
        else:  # robot2
            # Continuous circle
            msg.linear.x, msg.angular.z = 0.15, 0.3

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create controllers
        burger_controller = RobotController('burger')
        robot2_controller = RobotController('robot2')
        
        # Use MultiThreadedExecutor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(burger_controller)
        executor.add_node(robot2_controller)
        
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        if 'burger_controller' in locals():
            burger_controller.destroy_node()
        if 'robot2_controller' in locals():
            robot2_controller.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
