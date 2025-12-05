# robot_car/keyboard_mecanum_control.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist # 修改为TwistStamped
import termios
import tty
import sys
import select
import os

class MecanumKeyboardControl(Node):
    def __init__(self):
        super().__init__('mecanum_keyboard_control')
        
        # 发布TwistStamped消息
        self.publisher_ = self.create_publisher(
            Twist,  # 修改消息类型
            '/mecanum_controller/cmd_vel_unstamped',  # 修改话题名称
            10
        )
        
        # 麦轮控制参数
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s
        self.strafe_speed = 0.5   # m/s 横向移动速度
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('''
麦轮小车键盘控制说明 (TwistStamped版本):
---------------------------
    W - 前进
    S - 后退  
    A - 左转
    D - 右转
    Q - 左平移
    E - 右平移
    Z - 左前斜向
    C - 右前斜向
    X - 停止
空格键 - 急停
ESC/q - 退出
''')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    twist_stamped = Twist()  # 使用TwistStamped
                    
                    # 麦轮运动控制逻辑
                    if key == 'w':  # 前进
                        twist_stamped.linear.x = self.linear_speed
                    elif key == 's':  # 后退
                        twist_stamped.linear.x = -self.linear_speed
                    elif key == 'a':  # 左转
                        twist_stamped.angular.z = self.angular_speed
                    elif key == 'd':  # 右转
                        twist_stamped.angular.z = -self.angular_speed
                    elif key == 'q':  # 左平移
                        twist_stamped.linear.y = self.strafe_speed
                    elif key == 'e':  # 右平移
                        twist_stamped.linear.y = -self.strafe_speed
                    elif key == 'z':  # 左前斜向
                        twist_stamped.linear.x = self.linear_speed * 0.7
                        twist_stamped.linear.y = self.strafe_speed * 0.7
                    elif key == 'c':  # 右前斜向
                        twist_stamped.linear.x = self.linear_speed * 0.7
                        twist_stamped.ist.linear.y = -self.strafe_speed * 0.7
                    elif key == 'x' or key == ' ':  # 停止
                        twist_stamped.linear.x = 0.0
                        twist_stamped.linear.y = 0.0
                        twist_stamped.angular.z = 0.0
                    elif key == '\x1b' or key == 'q':  # ESC或q退出
                        break
                    
                    self.publisher_.publish(twist_stamped)
                    self.get_logger().info( f'控制命令: '
                                            f'linear.x={twist_stamped.linear.x:.2f}, '
                                            f'linear.y={twist_stamped.linear.y:.2f}, '
                                            f'angular.z={twist_stamped.angular.z:.2f}')
        
        finally:
            # 退出时发送停止命令
            twist_stamped = Twist()
            self.publisher_.publish(twist_stamped)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = MecanumKeyboardControl()
    
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'错误: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()