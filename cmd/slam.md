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

