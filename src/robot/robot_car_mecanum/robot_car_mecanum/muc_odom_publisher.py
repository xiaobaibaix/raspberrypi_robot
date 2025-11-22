#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TfOdomRelay(Node):
    def __init__(self):
        super().__init__('tf_odom_relay')
        # 订阅控制器发出来的备用话题
        self.sub = self.create_subscription(
            TransformStamped,
            '/mecanum_drive_controller/tf_odometry',
            self.tf_callback,
            10)
        # TF 广播器
        self.broadcaster = TransformBroadcaster(self)
        self.get_logger().info('Relay /mecanum_drive_controller/tf_odometry -> /tf')

    def tf_callback(self, msg: TransformStamped):
        self.get_logger().info(f'Relaying TF: {msg.header.frame_id} -> {msg.child_frame_id}')  
        msg.child_frame_id="base_footprint"  # 修改子坐标系名称
        # self.broadcaster.sendTransform(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TfOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()