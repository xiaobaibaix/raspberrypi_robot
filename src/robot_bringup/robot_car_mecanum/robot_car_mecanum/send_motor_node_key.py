#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import MotorsState, MotorState
import sys
import tty
import termios
import select

# 键位定义
KEY_MAP = {
    'q': (1,  120),   # 前轮1 正转
    'w': (2,  120),   # 前轮2 正转
    'a': (3,  120),   # 后轮1 正转
    's': (4,  120),   # 后轮2 正转
    'Q': (1, -120),   # 前轮1 反转
    'W': (2, -120),   # 前轮2 反转
    'A': (3, -120),   # 后轮1 反转
    'S': (4, -120),   # 后轮2 反转
    ' ': (0,    0)    # 空格急停
}

class MotorKeyTeleop(Node):
    def __init__(self):
        super().__init__('motor_key_teleop')
        self.pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        self.get_logger().info(
            "键盘控制: q/w 前轮±120rps, a/s 后轮±120rps, 空格急停, Ctrl-C 退出")
        self.send_zero()   # 上电先停

    def send_zero(self):
        zero = MotorsState(data=[MotorState(id=i, rps=0.0) for i in range(1, 5)])
        self.pub.publish(zero)

    def send_single(self, motor_id, rps):
        msg = MotorsState(data=[MotorState(id=motor_id, rps=float(rps))])
        self.pub.publish(msg)
        self.get_logger().info(f' motor_id={motor_id}  rps={rps}')

    def get_key(self):
        """非阻塞读一个字符，无键返回 None"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def run(self):
        # 设置终端为原始模式
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in KEY_MAP:
                    mid, rps = KEY_MAP[key]
                    if mid == 0:          # 空格
                        self.send_zero()
                    else:
                        self.send_single(mid, rps)
                elif key == '\x03':       # Ctrl-C
                    break
        finally:
            # 恢复终端设置 & 发送 0 速
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
            self.send_zero()
            self.get_logger().info('已退出，电机已停止')

def main():
    rclpy.init()
    node = MotorKeyTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()