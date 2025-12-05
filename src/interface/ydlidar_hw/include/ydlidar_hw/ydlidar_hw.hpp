#ifndef YDLIDAR_HW__YDLIDAR_HW_HPP_
#define YDLIDAR_HW__YDLIDAR_HW_HPP_

#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "src/CYdLidar.h"

namespace ydlidar_hw
{
class YdlidarHw : public hardware_interface::SensorInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time,const rclcpp::Duration & period) override;

private:
  CYdLidar lidar_;
  bool activated_ = false;
  /* 把所有 ROS 参数先缓存到成员，方便 on_configure 时一次性写给 SDK */
  struct Params {
    std::string port        = "/dev/ttyAMA0";
    std::string ignore_array= "";
    std::string frame_id    = "laser_frame";
    int         baudrate    = 115200;
    int         lidar_type  = 1;          // TYPE_TRIANGLE
    int         device_type = 0;          // YDLIDAR_TYPE_SERIAL
    int         sample_rate = 9;
    int         abnormal_check_count = 4;
    int         intensity_bit = 8;
    bool        fixed_resolution= false;
    bool        reversion       = true;
    bool        inverted        = true;
    bool        auto_reconnect  = true;
    bool        isSingleChannel = false;
    bool        intensity       = false;
    bool        support_motor_dtr= false;
    bool        invalid_range_is_inf = false;
    double      angle_max       = 180.0;
    double      angle_min       = -180.0;
    double      range_max       = 64.0;
    double      range_min       = 0.1;
    double      frequency       = 10.0;
  } params_;
  // 共享内存：一次扫下来的  angle_min, angle_max, ranges[]
  double angle_min_=0, angle_max_=0, range_min_=0, range_max_=0;
  double time_increment_=0, scan_time_=0;
  double point_count_=0;

  static constexpr size_t MAX_POINTS = 1440;
  std::vector<double> ranges_     {MAX_POINTS, 0.0};
  std::vector<double> intensities_{MAX_POINTS, 0.0};
};
}  // namespace ydlidar_hw

#endif