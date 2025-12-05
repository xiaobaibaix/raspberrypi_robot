#include "ydlidar_hw/ydlidar_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter.hpp"

namespace ydlidar_hw
{

    hardware_interface::CallbackReturn YdlidarHw::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        try
        {
            params_.port=params.hardware_info.hardware_parameters.at("port");
            params_.ignore_array = params.hardware_info.hardware_parameters.at("ignore_array");
            params_.frame_id = params.hardware_info.hardware_parameters.at("frame_id");
            params_.baudrate = std::stoi(params.hardware_info.hardware_parameters.at("baudrate"));
            params_.lidar_type = std::stoi(params.hardware_info.hardware_parameters.at("lidar_type"));
            params_.device_type = std::stoi(params.hardware_info.hardware_parameters.at("device_type"));
            params_.sample_rate = std::stoi(params.hardware_info.hardware_parameters.at("sample_rate"));
            params_.abnormal_check_count = std::stoi(params.hardware_info.hardware_parameters.at("abnormal_check_count"));
            params_.intensity_bit = std::stoi(params.hardware_info.hardware_parameters.at("intensity_bit"));
            params_.fixed_resolution = params.hardware_info.hardware_parameters.at("fixed_resolution") == "true";
            params_.reversion = params.hardware_info.hardware_parameters.at("reversion") == "true";
            params_.inverted = params.hardware_info.hardware_parameters.at("inverted") == "true";
            params_.auto_reconnect = params.hardware_info.hardware_parameters.at("auto_reconnect") == "true";
            params_.isSingleChannel = params.hardware_info.hardware_parameters.at("isSingleChannel") == "true";
            params_.intensity = params.hardware_info.hardware_parameters.at("intensity") == "true";
            params_.support_motor_dtr = params.hardware_info.hardware_parameters.at("support_motor_dtr") == "true";
            params_.invalid_range_is_inf = params.hardware_info.hardware_parameters.at("invalid_range_is_inf") == "true";
            params_.angle_max = std::stod(params.hardware_info.hardware_parameters.at("angle_max"));
            params_.angle_min = std::stod(params.hardware_info.hardware_parameters.at("angle_min"));
            params_.range_max = std::stod(params.hardware_info.hardware_parameters.at("range_max"));
            params_.range_min = std::stod(params.hardware_info.hardware_parameters.at("range_min"));
            params_.frequency = std::stod(params.hardware_info.hardware_parameters.at("frequency"));
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(get_logger(), "Missing parameter: %s", e.what());
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "YdlidarHw on_init OK, port=%s", params_.port.c_str());
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn YdlidarHw::on_configure(const rclcpp_lifecycle::State &)
    {
        /* 1. port & ignore array */
        lidar_.setlidaropt(LidarPropSerialPort, params_.port.c_str(), params_.port.size());
        lidar_.setlidaropt(LidarPropIgnoreArray, params_.ignore_array.c_str(), params_.ignore_array.size());

        /* 2. int 参数 */
        lidar_.setlidaropt(LidarPropSerialBaudrate, &params_.baudrate, sizeof(int));
        lidar_.setlidaropt(LidarPropLidarType, &params_.lidar_type, sizeof(int));
        lidar_.setlidaropt(LidarPropDeviceType, &params_.device_type, sizeof(int));
        lidar_.setlidaropt(LidarPropSampleRate, &params_.sample_rate, sizeof(int));
        lidar_.setlidaropt(LidarPropAbnormalCheckCount, &params_.abnormal_check_count, sizeof(int));
        lidar_.setlidaropt(LidarPropIntenstiyBit, &params_.intensity_bit, sizeof(int));

        /* 3. bool 参数 */
        lidar_.setlidaropt(LidarPropFixedResolution, &params_.fixed_resolution, sizeof(bool));
        lidar_.setlidaropt(LidarPropReversion, &params_.reversion, sizeof(bool));
        lidar_.setlidaropt(LidarPropInverted, &params_.inverted, sizeof(bool));
        lidar_.setlidaropt(LidarPropAutoReconnect, &params_.auto_reconnect, sizeof(bool));
        lidar_.setlidaropt(LidarPropSingleChannel, &params_.isSingleChannel, sizeof(bool));
        lidar_.setlidaropt(LidarPropIntenstiy, &params_.intensity, sizeof(bool));
        lidar_.setlidaropt(LidarPropSupportMotorDtrCtrl, &params_.support_motor_dtr, sizeof(bool));

        /* 4. float 参数 */
        float f_angle_max = static_cast<float>(params_.angle_max);
        float f_angle_min = static_cast<float>(params_.angle_min);
        float f_range_max = static_cast<float>(params_.range_max);
        float f_range_min = static_cast<float>(params_.range_min);
        float f_frequency = static_cast<float>(params_.frequency);
        lidar_.setlidaropt(LidarPropMaxAngle, &f_angle_max, sizeof(float));
        lidar_.setlidaropt(LidarPropMinAngle, &f_angle_min, sizeof(float));
        lidar_.setlidaropt(LidarPropMaxRange, &f_range_max, sizeof(float));
        lidar_.setlidaropt(LidarPropMinRange, &f_range_min, sizeof(float));
        lidar_.setlidaropt(LidarPropScanFrequency, &f_frequency, sizeof(float));

        /* 5. 初始化 SDK */
        if (!lidar_.initialize())
        {
            RCLCPP_ERROR(get_logger(), "YDLIDAR initialize failed: %s", lidar_.DescribeError());
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "YdlidarHw on_configure OK");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn YdlidarHw::on_activate(const rclcpp_lifecycle::State &)
    {
        activated_ = lidar_.turnOn();
        return activated_ ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
    }

    hardware_interface::CallbackReturn YdlidarHw::on_deactivate(const rclcpp_lifecycle::State &)
    {
        lidar_.turnOff();
        activated_ = false;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    YdlidarHw::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> states;

        states.emplace_back(hardware_interface::StateInterface(
            info_.name, "angle_min", &angle_min_));
        states.emplace_back(hardware_interface::StateInterface(
            info_.name, "angle_max", &angle_max_));
        states.emplace_back(hardware_interface::StateInterface(
            info_.name, "range_min", &range_min_));
        states.emplace_back(hardware_interface::StateInterface(
            info_.name, "range_max", &range_max_));
        states.emplace_back(hardware_interface::StateInterface(
            info_.name, "point_count", &point_count_));
        states.emplace_back(hardware_interface::StateInterface(
            info_.name, "time_increment", &time_increment_));
        states.emplace_back(hardware_interface::StateInterface(
            info_.name, "scan_time", &scan_time_));

        for (size_t i = 0; i < ranges_.size(); ++i)
            states.emplace_back(hardware_interface::StateInterface(
                info_.name, "range_" + std::to_string(i), &ranges_[i]));
        for (size_t i = 0; i < intensities_.size(); ++i)
            states.emplace_back(hardware_interface::StateInterface(
                info_.name, "intensity_" + std::to_string(i), &intensities_[i]));
        return states;
    }

    hardware_interface::return_type YdlidarHw::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (!activated_)
            return hardware_interface::return_type::OK;

        LaserScan scan;
        if (lidar_.doProcessSimple(scan))
        {
            angle_min_ = scan.config.min_angle;
            angle_max_ = scan.config.max_angle;
            range_min_ = scan.config.min_range;
            range_max_ = scan.config.max_range;
            time_increment_ = scan.config.time_increment;
            scan_time_ = scan.config.scan_time;

            point_count_ = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;

            for (size_t i = 0; i < scan.points.size(); ++i)
            {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
                if(index >=0 && index < point_count_) {
                    ranges_[i] = scan.points[i].range;
                    intensities_[i] = scan.points[i].intensity;
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

} // namespace ydlidar_hw
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ydlidar_hw::YdlidarHw, hardware_interface::SensorInterface)