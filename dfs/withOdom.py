#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

class TurnAutonomous(Node):

    def __init__(self):
        super().__init__('turn_autonomous')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)

        self.latest_distance = 10.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.home_x = None
        self.home_y = None
        
        self.last_print_time = 0.0
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("ANode online.")

    def odom_callback(self, msg):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        if self.home_x is None:
            self.home_x = raw_x
            self.home_y = raw_y

        # Swapping: Raw Y becomes our X, Raw X becomes our Y
        self.current_x = raw_y - self.home_y
        self.current_y = raw_x - self.home_x

    def scan_callback(self, msg):
        front_indices = msg.ranges[0:30] + msg.ranges[-30:]
        valid_ranges = [r for r in front_indices if 0.1 < r < 30.0]
        if valid_ranges:
            self.latest_distance = min(valid_ranges)

    def control_loop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.latest_distance < 0.8:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.5 
            self.get_logger().warn(f"OBSTACLE. Swapped Pos: [{self.current_x:.2f}, {self.current_y:.2f}]")
        else:
            msg.twist.linear.x = 0.15
            msg.twist.angular.z = 0.0
            
            if current_time - self.last_print_time >= 1.0:
                self.get_logger().info(f"Swapped X: {self.current_x:.2f}, Y: {self.current_y:.2f} | Dist: {self.latest_distance:.2f}m")
                self.last_print_time = current_time

        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = TurnAutonomous()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(TwistStamped())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
