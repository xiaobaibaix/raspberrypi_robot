from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
import subprocess
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # 声明启动参数
    ld.add_action(
        DeclareLaunchArgument(
            'robot_desc',
            default_value='',
            description='Robot description in URDF format',
        )
    )

    robot_desc = LaunchConfiguration('robot_desc')

    # 启动Gazebo仿真
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),  # 修正为字典格式
        )
    )

    # 在Gazebo中生成机器人模型
    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', robot_desc, 
                       '-x', '1.0', 
                       '-y', '1.0', 
                       '-z', '1.0', 
                       '-name', 'car'],
            output='screen'
        )
    )

    # 启动ROS-Gazebo桥接（只桥接仿真时钟）。
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/empty/clock', '/clock')
        ],
        output='screen'
    )
    ld.add_action(clock_bridge)
    
    return ld