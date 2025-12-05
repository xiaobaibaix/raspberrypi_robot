cmake_minimum_required(VERSION 3.8)
project(ydlidar_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------- 依赖 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

# --------- 控制器库 ----------
add_library(ydlidar_broadcaster SHARED
  src/ydlidar_broadcaster.cpp
)
target_include_directories(ydlidar_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ydlidar_broadcaster
  rclcpp
  controller_interface
  hardware_interface
  sensor_msgs
  realtime_tools
  pluginlib
  tf2_ros
)

# --------- 插件导出 ----------
pluginlib_export_plugin_description_file(controller_interface
  ydlidar_broadcaster_plugin.xml)

# --------- 安装 ----------
install(TARGETS ydlidar_broadcaster
  EXPORT export_ydlidar_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

install(FILES ydlidar_broadcaster_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# --------- 导出 ----------
ament_export_targets(export_ydlidar_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface hardware_interface sensor_msgs realtime_tools pluginlib)

# --------- 测试（可选） ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()