from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    urdfname = "robot_car.xacro"
    packagepath = get_package_share_directory("robot_car_mecanum")
    robot_desc = get_urdf_from_xacro('/'.join([packagepath, 'urdf', urdfname]))

    return LaunchDescription([
        # 将URDF模型加载到参数服务器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 启动joint_state_publisher（可选，用于手动调整关节）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ])

def get_urdf_from_xacro(xacro_file: str) -> str:
    result = subprocess.run(
        ["xacro", xacro_file],  # 修正命令名拼写
        stdout=subprocess.PIPE,
        text=True,
        check=True
    )
    return result.stdout