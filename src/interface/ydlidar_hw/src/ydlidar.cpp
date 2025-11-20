#include "ydlidar_hw/ydlidar.hpp"

namespace ydlidar_hw
{
    hardware_interface::CallbackReturn YdLidarHW::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=  // 注意：这里传入 params 而非 info
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> YdLidarHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> YdLidarHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        return command_interfaces;
    }
    
    hardware_interface::CallbackReturn YdLidarHW::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn YdLidarHW::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }   

    hardware_interface::return_type YdLidarHW::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type YdLidarHW::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }

}  // namespace ydlidar_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ydlidar_hw::YdLidarHW, hardware_interface::SystemInterface)