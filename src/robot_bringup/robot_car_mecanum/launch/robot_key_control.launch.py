# launch/keyboard_mecanum.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 1. 声明参数
    use_stamped = DeclareLaunchArgument(
        'use_stamped',
        default_value='True',
        description='True -> TwistStamped, False -> Twist'
    )

    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/mecanum_drive_controller/reference',
        description='Remap topic for cmd vel'
    )

    # 2. 节点
    keyboard_node = Node(
        package='robot_car_mecanum',
        executable='key_control',
        name='keyboard_mecanum_control',
        output='screen',
        parameters=[{'use_stamped': LaunchConfiguration('use_stamped')},
                    {'cmd_vel': LaunchConfiguration('cmd_vel_topic')}],
    )

    return LaunchDescription([
        use_stamped,
        cmd_vel_topic,
        keyboard_node
    ])