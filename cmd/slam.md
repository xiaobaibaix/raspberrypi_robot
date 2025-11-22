sudo apt install ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-turtlebot3*

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# 仿真
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
ros2 launch slam_toolbox online_async.launch.py queue_size:=30 scan_queue_size:=30

ros2 run rviz2 rviz2

ros2 run turtlebot3_teleop teleop_keyboard

ros2 run nav2_map_server map_saver_cli -f ~/maps/

state_pub_->publish(robot_msgs::msg::MotorsState(
    robot_msgs::msg::MotorsState().set__data({
        robot_msgs::msg::MotorState().set__id(1).set__rps(hw_commands_[0]*100),
        robot_msgs::msg::MotorState().set__id(2).set__rps(-hw_commands_[1]*100),
        robot_msgs::msg::MotorState().set__id(3).set__rps(-hw_commands_[2]*100),
        robot_msgs::msg::MotorState().set__id(4).set__rps(hw_commands_[3]*100),
    })
));
hw_velocities_ = hw_commands_;  // 简单模拟：命令即速度
hw_positions_  = hw_positions_; // 简单模拟：位置不变


const double dt = period.seconds();
for (size_t i = 0; i < info_.joints.size(); ++i)
{
    const auto & type = command_interface_types_[i];
    if (type == "velocity")
    {
        hw_velocities_[i] = hw_commands_[i];
        hw_positions_[i] += hw_velocities_[i] * dt;  // 简单积分模拟
    }
    else if (type == "position")
    {
        hw_positions_[i] = hw_commands_[i];
        hw_velocities_[i] = 0.0;
    }
    else
    {
        // 其他类型未模拟
        hw_velocities_[i] = 0.0;
    }
}