#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TurnAutonomous(Node):

    def __init__(self):
        super().__init__('turn_autonomous')

        # Sync with Gazebo clock to ensure timer intervals match simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # QoS (Quality of Service) profile: 
        # BEST_EFFORT is required for high-frequency sensor data like LiDAR in Gazebo
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber for LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Internal state variables
        self.latest_distance = 10.0  # Default safe distance
        self.last_print_time = 0.0   # Timestamp for throttled logging
        
        # Periodic control loop running at 10Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("online node")

    def scan_callback(self, msg):
        """
        Processes LiDAR scan messages.
        Filters for the front 60-degree cone of the robot.
        """
        # Slice the range array: indices 0-29 (left) and the last 30 indices (right)
        # In most LiDARs, index 0 is center-front
        front_indices = msg.ranges[0:30] + msg.ranges[-30:]
        
        # Filter out invalid readings:
        # Ignore values < 0.1 (self-collisions) and > 30.0 (sensor limit)
        valid_ranges = [r for r in front_indices if 0.1 < r < 30.0]
        
        if valid_ranges:
            # Store only the closest object detected in the front cone
            self.latest_distance = min(valid_ranges)

    def control_loop(self):
        """
        Reactive behavior: 
        If an obstacle is detected, stop and spin. Otherwise, move forward.
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Get current simulation time in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9

        # --- LOGIC: REACTIVE BEHAVIOR ---
        # Threshold: 0.8 meters
        if self.latest_distance < 0.8:
            # OBSTACLE DETECTED: Stop forward movement and rotate Z-axis
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.5 
            self.get_logger().warn(f"OBSTACLE Distance: {self.latest_distance:.2f}m - TURNING")
        else:
            # PATH CLEAR: Move forward at 0.15 m/s
            msg.twist.linear.x = 0.15
            msg.twist.angular.z = 0.0
            
            # Throttled Logging: Print distance once every 1.0 seconds
            if current_time - self.last_print_time >= 1.0:
                self.get_logger().info(f"SObject at: {self.latest_distance:.2f}m")
                self.last_print_time = current_time

        # Publish the velocity command
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = TurnAutonomous()
    try:
        # Keep the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # --- EMERGENCY STOP ---
        # When the script is killed, send one last 0 velocity command
        stop_msg = TwistStamped()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()