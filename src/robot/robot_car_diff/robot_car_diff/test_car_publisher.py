#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

class DiffcarTestNode(Node):
    def __init__(self):
        super().__init__('diffcar_control_node')
        # diff_drive_controller 默认订阅未加时间戳的话题：/diff_drive_controller/cmd_vel_unstamped (geometry_msgs/Twist)
        self.diff_publisher=self.create_publisher(TwistStamped,'/diff_drive_controller/cmd_vel',10)

        self.t1=self.create_timer(0.05,self.move_diff)
        self.get_logger().info('diffcar_control_node started.')
    
    def move_diff(self):
        msg=TwistStamped()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.twist.linear.x=0.5
        msg.twist.angular.z=0.3
        #self.get_logger().info('Publishing cmd_val: linear_x=%.2f, angular_x=%.2f' % (msg.twist.linear.x, msg.twist.angular.x))
        self.diff_publisher.publish(msg)

def  main(args=None):
    rclpy.init(args=args)
    node=DiffcarTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()