#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import termios, tty, select, sys, os

class MecanumKeyboardControl(Node):
    def __init__(self):
        super().__init__('mecanum_keyboard_control')

        # ---------------- 参数 ----------------
        self.use_stamped = self.declare_parameter('use_stamped', True).value
        self.topic_name  = self.declare_parameter('cmd_vel', '/mecanum_drive_controller/reference').value
        qos = 10

        # ---------------- 发布器 ----------------
        if self.use_stamped:
            self.pub = self.create_publisher(TwistStamped, self.topic_name, qos)
            self.get_logger().info('Publishing TwistStamped on "%s"' % self.topic_name)
        else:
            self.pub = self.create_publisher(Twist, self.topic_name, qos)
            self.get_logger().info('Publishing Twist on "%s"' % self.topic_name)

        # ---------------- 运动参数 ----------------
        self.linear_speed  = 0.3
        self.angular_speed = 0.3
        self.strafe_speed  = 0.3

        # ---------------- 终端检测 ----------------
        self.tty = sys.stdin.isatty()
        if self.tty:
            try:
                self.settings = termios.tcgetattr(sys.stdin)
            except termios.error:
                self.tty = False
                self.get_logger().warn('tcgetattr failed — run in dumb mode')
        else:
            self.get_logger().warn('stdin is NOT a tty — keyboard control disabled')
        self.settings = self.settings if self.tty else None
        self._last_len = 0   # 同一行刷新用

        self.print_help()

    # ------------------------------------------------------------------
    #  帮助
    # ------------------------------------------------------------------
    def print_help(self):
        self.get_logger().info('''
麦轮小车键盘控制说明
---------------------------
  W - 前进   S - 后退
  A - 左转   D - 右转
  Q - 左平移 E - 右平移
  Z - 左前斜 C - 右前斜
  X/Space - 停止
  ESC/q   - 退出
---------------------------
        ''')

    # ------------------------------------------------------------------
    #  读键（无 tty 立即返回空）
    # ------------------------------------------------------------------
    def get_key(self):
        if not self.tty or self.settings is None:
            return ''
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            key = sys.stdin.read(1) if rlist else ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except termios.error:
            return ''

    # ------------------------------------------------------------------
    #  同一行刷新显示（stdout）
    # ------------------------------------------------------------------
    def print_inline(self, text):
        sys.stdout.write('\r' + ' ' * self._last_len + '\r' + text)
        sys.stdout.flush()
        self._last_len = len(text)

    # ------------------------------------------------------------------
    #  构造消息
    # ------------------------------------------------------------------
    def create_twist(self):
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            return msg
        return Twist()

    # ------------------------------------------------------------------
    #  主循环
    # ------------------------------------------------------------------
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = self.create_twist()
                t = msg.twist if self.use_stamped else msg   # 统一句柄

                if key == 'w':
                    t.linear.x = self.linear_speed
                elif key == 's':
                    t.linear.x = -self.linear_speed
                elif key == 'a':
                    t.angular.z = self.angular_speed
                elif key == 'd':
                    t.angular.z = -self.angular_speed
                elif key == 'q':
                    t.linear.y = self.strafe_speed
                elif key == 'e':
                    t.linear.y = -self.strafe_speed
                elif key == 'z':
                    t.linear.x = self.linear_speed * 0.7
                    t.linear.y = self.strafe_speed * 0.7
                elif key == 'c':
                    t.linear.x = self.linear_speed * 0.7
                    t.linear.y = -self.strafe_speed * 0.7
                elif key in ('x', ' '):
                    t.linear.x = t.linear.y = t.angular.z = 0.0
                elif key in ('\x1b', 'q'):
                    break
                else:
                    continue

                self.pub.publish(msg)
                # 同一行刷新
                self.print_inline(
                    f'控制命令: linear.x={t.linear.x:.2f}, '
                    f'linear.y={t.linear.y:.2f}, angular.z={t.angular.z:.2f}'
                )

        except KeyboardInterrupt:
            pass
        finally:
            # 发停止 & 恢复终端
            stop = self.create_twist()
            (stop.twist if self.use_stamped else stop).linear.x  = 0.0
            (stop.twist if self.use_stamped else stop).linear.y  = 0.0
            (stop.twist if self.use_stamped else stop).angular.z = 0.0
            self.pub.publish(stop)

            if self.tty and self.settings is not None:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                except termios.error:
                    pass
            sys.stdout.write('\n')
            sys.stdout.flush()


# --------------------------- main ----------------------------
def main():
    rclpy.init()
    node = MecanumKeyboardControl()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()