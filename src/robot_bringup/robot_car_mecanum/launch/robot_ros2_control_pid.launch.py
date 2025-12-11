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
            # 添加 use_sim_time 参数以避免警告
            'use_sim_time': False,
        }],
        output='both'
    )
    ld.add_action(robot_state_publisher)

    # ① 硬件插件 + 控制器管理器（必须同机）
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg, 'config', 'controllers_pid.yaml'),
            # 添加 use_sim_time 参数
            {'use_sim_time': False},
        ],
        output='both',
    )
    ld.add_action(controller_manager)

    # 使用更可靠的事件处理方式
    jspawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['joint_state_broadcaster'],
        output='both',
        # 添加前缀避免命名空间问题
        prefix='',
    )

    imuspawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['imu_sensor_broadcaster'],
        output='both',
        prefix='',
    )

    # PID控制器按顺序启动而不是同时启动
    fl_pid_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['fl_speed_pid'],
        output='both',
        prefix='',
    )
    
    fr_pid_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['fr_speed_pid'],
        output='both',
        prefix='',
    )
    
    rl_pid_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['rl_speed_pid'],
        output='both',
        prefix='',
    )
    
    rr_pid_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['rr_speed_pid'],
        output='both',
        prefix='',
    )

    mecaspawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['mecanum_drive_controller'],
        output='both',
        prefix='',
        # 为麦克纳姆控制器添加额外参数
        parameters=[{'use_sim_time': False}]
    )

    # 修改事件处理顺序
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(
                    period=0.0,
                    actions=[jspawner]
                )
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jspawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[imuspawner]
                )
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            # target_action=imuspawner,
            target_action=jspawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[fl_pid_spawner]
                )
            ]
        )
    ))

    # 顺序启动PID控制器
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fl_pid_spawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[fr_pid_spawner]
                )
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fr_pid_spawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[rl_pid_spawner]
                )
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rl_pid_spawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[rr_pid_spawner]
                )
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rr_pid_spawner,
            on_exit=[
                TimerAction(
                    period=0.0,
                    actions=[mecaspawner]
                )
            ]
        )
    ))

    return ld