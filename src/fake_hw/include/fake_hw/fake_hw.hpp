#pragma once
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/subscription.hpp"

namespace fake_hw
{
    class FakeHW : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FakeHW)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams & params) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    private:
        // 多关节状态与命令缓存，大小由 info_.joints.size() 决定
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_commands_;
        // 记录每个关节使用的命令接口类型（如 "position" 或 "velocity"）
        std::vector<std::string> command_interface_types_;

        // 2. 加订阅句柄
        // rclcpp::Subscription<fake_hw::msg::WheelStates>::SharedPtr wheel_sub_;

        // 3. 回调
        // void on_wheel_states(const fake_hw::msg::WheelStates::SharedPtr msg);
    };
}
