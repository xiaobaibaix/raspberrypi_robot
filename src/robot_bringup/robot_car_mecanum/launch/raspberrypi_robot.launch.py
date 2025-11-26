from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('robot_car_mecanum')

    # ① 硬件插件 + 控制器管理器（必须同机）
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg, 'config', 'controllers.yaml')],
        output='both'
    )

    # ② 硬件相关控制器（ spawner 可远程，但这里放本地也可）
    jspawner   = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['joint_state_broadcaster'],
        output='both'
        )
    diffspawner= Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['mecanum_controller'],
        output='both'
        )
    imuspawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['imu_sensor_broadcaster'],
        output='both'
        )

    return LaunchDescription([
        controller_manager,
        jspawner,
        diffspawner,
        imuspawner
    ])