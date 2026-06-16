#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_to_quat(yaw_deg):
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('publish_initial_pose')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.count = 0

        self.x = -0.373
        self.y = -0.222
        self.yaw_deg = 31.2

    def timer_cb(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        z, w = yaw_to_quat(self.yaw_deg)
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w

        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10

        self.pub.publish(msg)
        self.count += 1
        self.get_logger().warn(
            f'Published initial pose #{self.count}: x={self.x}, y={self.y}, yaw={self.yaw_deg}'
        )
        if self.count >= 3:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
