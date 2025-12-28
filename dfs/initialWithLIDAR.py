#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TurnAutonomous(Node):

    def __init__(self):
        super().__init__('turn_autonomous')

        # Sync with Gazebo clock
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        self.latest_distance = 10.0
        self.last_print_time = 0.0
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("online node")

    def scan_callback(self, msg):
        # front 60 degrees
        front_indices = msg.ranges[0:30] + msg.ranges[-30:]
        valid_ranges = [r for r in front_indices if 0.1 < r < 30.0]
        
        if valid_ranges:
            self.latest_distance = min(valid_ranges)

    def control_loop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Get current time in seconds for 1-second interval
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Check for obstacle
        if self.latest_distance < 0.8:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.5 
            self.get_logger().warn(f"OBSTACLE Distance: {self.latest_distance:.2f}m - TURNING")
        else:
            msg.twist.linear.x = 0.15
            msg.twist.angular.z = 0.0
            
            # Print distance only once every 1 second
            if current_time - self.last_print_time >= 1.0:
                self.get_logger().info(f"SObject at: {self.latest_distance:.2f}m")
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
        # Emergency stop on exit
        stop_msg = TwistStamped()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
