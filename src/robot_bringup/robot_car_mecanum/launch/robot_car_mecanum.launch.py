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

    # 获取URDF描述（在运行时通过 xacro 命令并传入 use_gazebo 参数）
    packagepath = get_package_share_directory("robot_car_mecanum")
    robot_description = Command([
        'xacro',
        ' ',
        PathJoinSubstitution([packagepath, 'urdf', 'robot_car.xacro']),
        ' ',
        TextSubstitution(text='use_gazebo:='),
        use_gazebo
    ])

    # Gazebo仿真启动（条件性）
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    packagepath,
                    'launch',
                    'car_gazebo.launch.py'
                )
            ]),
            launch_arguments={'robot_desc': robot_description}.items(),
            condition=IfCondition(use_gazebo)
        )
    )

    # 机器人状态发布者节点
    robot_desc_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_gazebo},
            {'robot_description': ParameterValue(robot_description, value_type=str)}
        ]
    )
    ld.add_action(robot_desc_node)

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(packagepath, 'config', 'robot_car.rviz')],
    )
    ld.add_action(rviz_node)

    # 控制器管理节点（条件性）
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(os.path.join(packagepath, 'config', 'controllers.yaml'), allow_substs=True)
        ],
        output="both",
        condition=UnlessCondition(use_gazebo)
    )

    ld.add_action(controller_manager_node)
    
    gazebo_jspawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gazebo_diffspawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gazebo_imuspawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    # 修复后的代码
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=0.0,
                    actions=[gazebo_jspawner]
                )
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_jspawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[gazebo_diffspawner]
                )
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_diffspawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[gazebo_imuspawner]
                )
            ]
        )
    ))

    return ld