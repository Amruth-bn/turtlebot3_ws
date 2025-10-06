#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math 

class OdomCovariancePatch(Node):
    def __init__(self):
        super().__init__('odom_covariance_patch')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/odom_patched', 10)

        # Improved covariances for a differential drive robot:
        # x, y: 0.05 (measured)
        # z, roll, pitch: 1.0 (not measured)
        # yaw: 0.1 (measured)
        # vx: 0.05 (measured)
        # vy, vz, vroll, vpitch: 1.0 (not measured)
        # vyaw: 0.1 (measured)
        self.pose_cov = [
            0.5, 0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.5, 0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    1.0,  0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    1.0,  0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    1.0,  0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.1
        ]
        self.twist_cov = [
            0.5, 0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.5, 0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    1.0,  0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    1.0,  0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    1.0,  0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.1
        ]

    def is_valid(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        vals = [p.x, p.y, p.z, o.x, o.y, o.z, o.w, t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
        return all(math.isfinite(v) for v in vals)

    def odom_callback(self, msg):
        if self.is_valid(msg):
            msg.pose.covariance = self.pose_cov
            msg.twist.covariance = self.twist_cov
            self.get_logger().info(f"Publishing odom with covariance: {msg.pose.covariance}")
        else:
            self.get_logger().warn("Invalid odometry data received, not publishing.")
            msg.pose.covariance = [0.0] * 36
            msg.twist.covariance = [0.0] * 36
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomCovariancePatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
