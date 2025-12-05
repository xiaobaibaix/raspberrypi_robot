#include "ydlidar_broadcaster/ydlidar_broadcaster.hpp"
#include "controller_interface/helpers.hpp"
#include "rclcpp/parameter.hpp"
#include <realtime_tools/realtime_publisher.hpp>
#include <limits>

namespace ydlidar_broadcaster
{

controller_interface::CallbackReturn
YdlidarBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("frame_id", "laser_frame");
    auto_declare<std::string>("base_frame_id", "base_link");
  } catch (const std::exception & e) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
YdlidarBroadcaster::command_interface_configuration() const
{
  return {};
}

controller_interface::InterfaceConfiguration
YdlidarBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  conf.names.emplace_back("ydlidar/angle_min");
  conf.names.emplace_back("ydlidar/angle_max");
  conf.names.emplace_back("ydlidar/range_min");
  conf.names.emplace_back("ydlidar/range_max");
  conf.names.emplace_back("ydlidar/point_count");
  conf.names.emplace_back("ydlidar/time_increment");
  conf.names.emplace_back("ydlidar/scan_time");

  for (size_t i = 0; i < 1440; ++i) {
    conf.names.emplace_back("ydlidar/range_"      + std::to_string(i));
    conf.names.emplace_back("ydlidar/intensity_"  + std::to_string(i));
  }
  return conf;
}

controller_interface::CallbackReturn
YdlidarBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  frame_id_   = get_node()->get_parameter("frame_id").as_string();
  base_frame_ = get_node()->get_parameter("base_frame_id").as_string();

  scan_pub_    = get_node()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  rt_scan_pub_ = std::make_unique<
                   realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>>(scan_pub_);

  rt_scan_pub_->msg_.header.frame_id = frame_id_;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
YdlidarBroadcaster::update(const rclcpp::Time & time,
                            const rclcpp::Duration &)
{
  if (rt_scan_pub_->trylock()) {
    auto & msg = rt_scan_pub_->msg_;
    msg.header.stamp = time;

    /*  取 optional，如果无效就用 NaN 占位  */
    const auto angle_min     = state_interfaces_[0].get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
    const auto angle_max     = state_interfaces_[1].get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
    const auto range_min     = state_interfaces_[2].get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
    const auto range_max     = state_interfaces_[3].get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
    const auto point_count_d = state_interfaces_[4].get_optional<double>().value_or(0.0);
    const auto time_inc      = state_interfaces_[5].get_optional<double>().value_or(0.0);
    const auto scan_time     = state_interfaces_[6].get_optional<double>().value_or(0.0);

    size_t point_count = static_cast<size_t>(point_count_d);

    msg.angle_min      = angle_min;
    msg.angle_max      = angle_max;
    msg.range_min      = range_min;
    msg.range_max      = range_max;
    msg.time_increment = time_inc;
    msg.scan_time      = scan_time;

    msg.ranges.resize(point_count);
    msg.intensities.resize(point_count);

    for (size_t i = 0; i < point_count; ++i) {
      msg.ranges[i]      = state_interfaces_[7 + 2*i].get_optional<double>().value_or(std::numeric_limits<float>::quiet_NaN());
      msg.intensities[i] = state_interfaces_[8 + 2*i].get_optional<double>().value_or(0.0f);
    }

    rt_scan_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace ydlidar_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ydlidar_broadcaster::YdlidarBroadcaster,
                       controller_interface::ControllerInterface)