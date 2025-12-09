
# ros2_control

| 查看所有已加载控制器              `ros2 control list_controllers`
| 查看硬件接口列表                  `ros2 control list_hardware_interfaces`
| 实时显示控制器状态                `ros2 control list_controllers --watch`
| 启动/停止/重启控制器              `ros2 control switch_controllers --start joint_state_broadcaster --stop forward_position_controller`
| 加载新控制器（不重启节点）         `ros2 control load_controller <name>`
| 卸载控制器                        `ros2 control unload_controller <name>`
| 把控制器参数写到 YAML 并保存       `ros2 control list_controllers -v`
| 可视化控制链                      `ros2 control view_controller_chains`
