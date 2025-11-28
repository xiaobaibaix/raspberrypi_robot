# ros2
colcon build --symlink-install --allow-overriding mecanum_drive_controller --packages-select 

sudo apt update && sudo apt install -y \
    ninja-build \
    build-essential \
    gcc \
    g++ \
    cmake \
    ccache


colcon build --symlink-install --parallel-workers $(nproc) \
             --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release \
                          -DBUILD_TESTING=OFF \
                          -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

ros2 pkg create --build-type ament_python --license MIT  <package_name> [options]
ros2 pkg create --build-type ament_cmake --license MIT  <package_name> [options]

# gazebo
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic

colcon build --symlink-install --packages-select name

# 礼貌通知 ROS 节点自杀
ros2 node list 2>/dev/null | xargs -I {} ros2 node kill {} 2>/dev/null

# 硬杀 Gazebo + ROS + Ruby 残留
pkill -9 -f "gz sim"
pkill -9 -f "gazebo"
pkill -9 -f "ruby"

pkill -9 -f "slam_toolbox"
pkill -9 -f "rviz2"

# 确认已清空
ros2 node list 2>/dev/null || echo "All ROS nodes killed"

ros2 topic info /tf -v | grep "Node name"