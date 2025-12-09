from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
import os

def generate_launch_description():
    ld = LaunchDescription()

    pkg = get_package_share_directory('robot_car_mecanum')

    # 加载机器人描述文件
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', os.path.join(pkg, 'urdf', 'robot_car.xacro')]),
                value_type=str
            ),
        }],
        output='both'
    )
    ld.add_action(robot_state_publisher)

    # ① 硬件插件 + 控制器管理器（必须同机）
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg, 'config', 'controllers.yaml'),
            {'use_realtime': False},   # 关闭实时调度
            {'lock_memory': False},    # 关闭内存锁定
        ],
        output='both',
    )
    ld.add_action(controller_manager)

    # ② 硬件相关控制器（ spawner 可远程，但这里放本地也可）
    jspawner   = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['joint_state_broadcaster'],
        output='both'
        )

    mecaspawner= Node(
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
    # laserspawner = Node(
    #     package='controller_manager', 
    #     executable='spawner', 
    #     arguments=['laser_broadcaster'],
    #     output='both'
    #     )
    
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                jspawner
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jspawner,
            on_exit=[
                mecaspawner
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mecaspawner,
            on_exit=[
                imuspawner
            ]
        )
    ))

    return ld