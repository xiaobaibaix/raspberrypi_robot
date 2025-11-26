#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math

class MecanumRefPublisher(Node):
    def __init__(self):
        super().__init__('mecanum_ref_publisher')
        self.pub = self.create_publisher(TwistStamped, '/mecanum_drive_controller/reference', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Starting to publish on /mecanum_drive_controller/reference')

    def timer_callback(self):
        msg = TwistStamped()
        # 打时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'  # 可改 odom/map

        # 车体坐标系下的速度命令（单位：m/s 和 rad/s）
        msg.twist.linear.x = 0.0   # 前进
        msg.twist.linear.y = 0.0   # 左横移（负值右横移）
        msg.twist.angular.z = 0.5   # 逆时针自转

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumRefPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()