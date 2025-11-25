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

#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "example_interfaces/srv/trigger.hpp"

#include "robot_msgs/msg/motors_state.hpp"   // 自定义消息
#include "std_msgs/msg/float64_multi_array.hpp"  // 临时演示用
#include "example_interfaces/srv/trigger.hpp"
#include "robot_msgs/srv/get_pwm_servo_state.hpp"  // 自定义服务

namespace mecanum_wheel_chassis_hw
{
    class MecanumWheelChassisHW : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MecanumWheelChassisHW)

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

        /* ==========  借用的 node 接口  ========== */
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
        rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
        rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;

        /* ==========  发布 / 服务  ========== */
        rclcpp::Publisher<robot_msgs::msg::MotorsState>::SharedPtr state_pub_;
    };
}
