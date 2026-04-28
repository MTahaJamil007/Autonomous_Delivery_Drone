# Loaction at which this file is located: "nano ~/ros2_ws/src/obstacle_avoider/obstacle_avoider/avoider_node.py"
# Code:
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import math

UDP_IP = "127.0.0.1"
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class LidarAvoider(Node):
    def __init__(self):
        super().__init__('lidar_avoider')
        self.subscription = self.create_subscription(LaserScan, '/lidar/scan', self.listener_callback, 10)
        self.current_dodge_direction = None
        self.get_logger().info("✅ ROS 2 Velocity Blending Avoider Online! Port 5006")

    def listener_callback(self, msg):
        clean_ranges = [r if not math.isinf(r) and not math.isnan(r) and r > 0.1 else 12.0 for r in msg.ranges]
        
        front_ranges = clean_ranges[0:30] + clean_ranges[-30:]
        left_ranges = clean_ranges[30:90]
        right_ranges = clean_ranges[-90:-30]
        
        min_front = min(front_ranges) if front_ranges else 12.0
        min_left = min(left_ranges) if left_ranges else 12.0
        min_right = min(right_ranges) if right_ranges else 12.0
        
        SAFE_DIST = 7.0
        CLEAR_DIST = 10.0
        
        if min_front < SAFE_DIST:
            if self.current_dodge_direction is None:
                self.current_dodge_direction = "DODGE_LEFT" if min_left > min_right else "DODGE_RIGHT"
            action = self.current_dodge_direction
            self.get_logger().warn(f"OBSTACLE: {min_front:.1f}m! LOCKED: {action}")
            
        elif self.current_dodge_direction is not None and min_front >= CLEAR_DIST:
            self.current_dodge_direction = None
            action = "CLEAR"
        else:
            action = self.current_dodge_direction if self.current_dodge_direction else "CLEAR"
            
        sock.sendto(action.encode(), (UDP_IP, UDP_PORT))

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()