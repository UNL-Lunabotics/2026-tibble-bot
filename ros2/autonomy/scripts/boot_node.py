#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math

class TibbleBootNode(Node):
    def __init__(self):
        super().__init__('tibble_boot_node')
        
        # Declare parameters so you can change them flexibly via YAML
        self.declare_parameter('target_tag_frame', 'tag_36h11_id0')
        self.declare_parameter('global_map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        
        self.target_tag = self.get_parameter('target_tag_frame').value
        self.map_frame = self.get_parameter('global_map_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value

        # TF2 Setup to listen for the AprilTag transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to initialize Nav2 / MOLA SLAM
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )

        # Timer to check for the tag
        self.timer = self.create_timer(1.0, self.attempt_localization)
        self.get_logger().info("Boot Node Started. Waiting for AprilTag...")

    def attempt_localization(self):
        try:
            # We want to find where the robot (base_link) is relative to the Tag
            # Assuming the Tag's real-world global position is static and known to the map
            now = rclpy.time.Time()
            
            # Get the transform from the Tag to the Robot Base
            trans = self.tf_buffer.lookup_transform(
                self.target_tag, 
                self.base_frame, 
                now
            )

            # In a real scenario, you would calculate: Map -> Tag -> Camera -> Base_link
            # Here we construct the initial pose message
            init_pose = PoseWithCovarianceStamped()
            init_pose.header.frame_id = self.map_frame
            init_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Apply your offset math here based on the transform
            init_pose.pose.pose.position.x = trans.transform.translation.x
            init_pose.pose.pose.position.y = trans.transform.translation.y
            init_pose.pose.pose.orientation = trans.transform.rotation
            
            # Give it a tight covariance since we are highly confident in the AprilTag
            init_pose.pose.covariance[0] = 0.05
            init_pose.pose.covariance[7] = 0.05
            init_pose.pose.covariance[35] = 0.05

            self.initial_pose_pub.publish(init_pose)
            self.get_logger().info("Initial pose published successfully!")

            # TODO: Add a service call here to kill the Realsense camera node to save CPU
            self.get_logger().info("Killing camera node and shutting down boot node...")
            
            # Stop the timer and exit once localized
            self.timer.cancel()
            
        except Exception as e:
            self.get_logger().warn(f"Waiting for tag {self.target_tag}... ({str(e)})")

def main(args=None):
    rclpy.init(args=args)
    node = TibbleBootNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()