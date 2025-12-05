// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#ifndef YDLIDAR_BROADCASTER__YDLIDAR_BROADCASTER_HPP_
#define YDLIDAR_BROADCASTER__YDLIDAR_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <realtime_tools/realtime_publisher.hpp>

namespace ydlidar_broadcaster
{
class YdlidarBroadcaster : public controller_interface::ControllerInterface
{
public:
  YdlidarBroadcaster()=default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::string frame_id_{"laser_link"};
  std::string base_frame_{"base_link"};

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>> rt_scan_pub_;
};
}  // namespace ydlidar_broadcaster
#endif  // YDLIDAR_BROADCASTER__YDLIDAR_BROADCASTER_HPP_