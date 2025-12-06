sudo apt install ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-turtlebot3*

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# toobox
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True


# save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/