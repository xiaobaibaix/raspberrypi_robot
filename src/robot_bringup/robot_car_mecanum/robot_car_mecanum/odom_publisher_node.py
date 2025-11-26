#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.br = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.01, self.timer_cb)   # 100 Hz
        self.get_logger().info('Fake odom running at 100 Hz (all zeros)')

    def timer_cb(self):
        now = self.get_clock().now()

        # 1. 发布 nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        # 其余字段默认就是 0，不用填
        self.pub.publish(odom)

        # 2. 发布 tf2  transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        # translation & rotation 默认全 0
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()