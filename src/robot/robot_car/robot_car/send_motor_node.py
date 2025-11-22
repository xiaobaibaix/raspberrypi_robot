#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import MotorsState, MotorState
import random
import time

class MotorLoop(Node):
    def __init__(self):
        super().__init__('motor_loop_publisher')
        self.pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('0.5-Hz motor loop started')

    def timer_callback(self):
        msg = MotorsState()
        msg.data = [
            MotorState(id=1, rps=random.uniform(-120.0, 120.0)),
            MotorState(id=2, rps=random.uniform(-120.0, 120.0)),
            MotorState(id=3, rps=random.uniform(-120.0, 120.0)),
            MotorState(id=4, rps=random.uniform(-120.0, 120.0))
        ]
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {[m.rps for m in msg.data]}')

    def stop_motors(self):
        if self.context.ok() and self.pub:
            zero = MotorsState(data=[MotorState(id=i, rps=0.0) for i in range(1,5)])
            self.pub.publish(zero)
            self.get_logger().info('Sent zero-speed before shutdown')
            # 给网络/串口 10 ms 时间把包送走
            time.sleep(0.1)

def main():
    rclpy.init()
    node = MotorLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C pressed')
        # 保证节点还活着，先发 0 速
        node.stop_motors()
    finally:
        # 此时不再发消息，只做清理
        node.destroy_node()
        rclpy.shutdown()