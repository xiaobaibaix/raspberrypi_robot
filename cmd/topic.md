# topic
//发布话题velocity_controller/commands 类型std_msgs/msg/Float64MultiArray
ros2 topic pub --rate 5 velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0,]}" 

# topic
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"