#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class DynamicPoseToOdom(Node):

    def __init__(self):
        super().__init__("dynamic_pose_to_odom")

        self.odom_frame = "odom"
        self.base_frame = "base_link"

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_sub = self.create_subscription(
            PoseArray,
            "/model/pet_bot_test5/pose",
            self.pose_callback,
            10
        )

        self.get_logger().info("Dynamic Pose -> Odometry node started")

    def pose_callback(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().warn("Empty PoseArray received!")
            return

        self.get_logger().info("Received pose -> Broadcasting odom TF")

        pose = msg.poses[0]
        now = self.get_clock().now().to_msg()

        # Publish /odom
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose
        self.odom_pub.publish(odom)

        # Publish TF: odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = DynamicPoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    