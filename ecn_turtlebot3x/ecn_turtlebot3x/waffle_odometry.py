import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class WaffleOdometry(Node):
    def __init__(self,):
        super().__init__('waffle_odometry')

        self.wheel_radius = 0.033
        self.wheel_seperation = 0.288
        self.prev_right_pose, self.prev_left_pose = 0.0, 0.0
        self.x, self.y, self.theta = -2.0, -0.5, 0.0

        # Set a configurable namespace arguament from ros-args
        self.declare_parameter('namespace', 'waffle')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        if self.namespace == '':
            self.get_logger().warn("Running without a namespace!\n")
            self.js_topic_name = 'joint_states'
            self.odom_topic_name = 'odom'
            self.base_link_name = 'base_footprint'
        else:
            self.get_logger().info(f"Running with namespace: {self.namespace}\n")
            self.js_topic_name = f'joint_states'
            self.odom_topic_name = f'{self.namespace}/odom'
            self.base_link_name = f'{self.namespace}/base_footprint'
        
        # declare joint_states subscriber
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            # f"/{self.js_topic_name}",
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            f"/{self.odom_topic_name}",
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.stf_brosdcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.pub_timer = self.create_timer(0.01, self.pub_odom_and_tf)
        self.get_logger().info(f'Waffle odometry node has been started.\n')
    
    def joint_states_callback(self, msg):
        right_pose = msg.position[1]
        left_pose = msg.position[0]
        
        delta_right = right_pose - self.prev_right_pose
        delta_left = left_pose - self.prev_left_pose

        self.prev_right_pose = right_pose
        self.prev_left_pose = left_pose

        delta_right_distance = delta_right * self.wheel_radius
        delta_left_distance = delta_left * self.wheel_radius
        
        delta_distance = (delta_right_distance + delta_left_distance) / 2.0
        delta_angle = (delta_right_distance - delta_left_distance) / self.wheel_seperation

        # Update the robot's position and orientation
        self.x += delta_distance * math.cos(self.theta + (delta_angle / 2.0))
        self.y += delta_distance * math.sin(self.theta + (delta_angle / 2.0))
        self.theta += delta_angle

    def pub_odom_and_tf(self):
        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_topic_name
        odom_msg.child_frame_id = self.base_link_name

        # Set the position and orientation
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        # quaternion = tf2_ros.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        # odom_msg.pose.pose.orientation.x = quaternion[0]
        # odom_msg.pose.pose.orientation.y = quaternion[1]
        # odom_msg.pose.pose.orientation.z = quaternion[2]
        # odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

        # Create and publish the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_topic_name
        t.child_frame_id = self.base_link_name
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # t.transform.rotation.x = quaternion[0]
        # t.transform.rotation.y = quaternion[1]
        # t.transform.rotation.z = quaternion[2]
        # t.transform.rotation.w = quaternion[3]

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        # Publish static transform between {namespace}/odom to odom
        # static_transform = TransformStamped()
        # static_transform.header.stamp = self.get_clock().now().to_msg()
        # static_transform.header.frame_id = 'odom'
        # static_transform.child_frame_id = self.odom_topic_name
        # static_transform.transform.translation.x = 0.0
        # static_transform.transform.translation.y = 0.0
        # static_transform.transform.translation.z = 0.0
        # static_transform.transform.rotation.x = 0.0
        # static_transform.transform.rotation.y = 0.0
        # static_transform.transform.rotation.z = 0.0
        # static_transform.transform.rotation.w = 1.0
        
        # self.stf_brosdcaster.sendTransform(static_transform)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = WaffleOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()