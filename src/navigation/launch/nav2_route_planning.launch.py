import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'controllers.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'recovery.yaml')
    smoother_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'smoother.yaml')

    
    return LaunchDescription([     
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[
                ('/cmd_vel', '/nav2_controller_raw_cmd')  # 全局名称要写全 /
            ],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            remappings=[
                ('/cmd_vel', '/nav2_controller_raw_cmd')  # 全局名称要写全 /
            ],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        # Node(
        #     package='nav2_smoother',
        #     executable='smoother_server',
        #     name='smoother_server',
        #     output='screen',
        #     parameters=[smoother_yaml],   # 你的 yaml 里已经包含 smoother_server 段
        # ),

        # Node(
        #     package='nav2_velocity_smoother',
        #     executable='velocity_smoother',
        #     name='velocity_smoother',
        #     output='screen',
        #     parameters=[smoother_yaml,
        #                 {'use_stamped_vel': True},        # 启用 stamped 输出
        #                 {'velocity_frame_id': 'base_link'}],  # header.frame_id
        #     remappings=[
        #         ('/cmd_vel_in',  '/nav2_controller_raw_cmd'),      # Twist 输入
        #         ('/cmd_vel_smooth_stamped', '/mecanum_drive_controller/reference')  # TwistStamped 输出
        #     ],
        # ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator',
                                        # 'smoother_server',
                                        # 'velocity_smoother'
                                        ]}],)
    ])
