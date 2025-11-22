from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import Node as LaunchNode
import subprocess
import os

from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():

    ld = LaunchDescription()

    # 声明启动参数
    ld.add_action(
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='whether to use Gazebo simulation',
            choices=['true','false','True','False']
        )
    )

    use_gazebo = LaunchConfiguration('use_gazebo', default='false')
    
    packagepath = get_package_share_directory("robot_car")

    # RViz2可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(packagepath, 'config', 'robot_rviz.rviz')],
        parameters=[
            {'use_sim_time': use_gazebo},
        ]
    )

    ld.add_action(rviz_node)


    return ld