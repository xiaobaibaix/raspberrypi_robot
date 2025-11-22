#include "four_wheel_chassis_hw/four_wheel_chassis.hpp"
#include <vector>
#include <string>
#include "robot_msgs/msg/motor_state.hpp"   // 自定义消息
namespace four_wheel_chassis_hw
{
    hardware_interface::CallbackReturn FourWheelChassisHW::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=  // 注意：这里传入 params 而非 info
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 后续逻辑完全不变（info_ 仍会被基类初始化，无需修改）
        const size_t n = info_.joints.size();
        hw_positions_.assign(n, 0.0);
        hw_velocities_.assign(n, 0.0);
        hw_commands_.assign(n, 0.0);
        command_interface_types_.resize(n);

        for (size_t i = 0; i < n; ++i)
        {
            if (!info_.joints[i].command_interfaces.empty())
            {
                command_interface_types_[i] = info_.joints[i].command_interfaces[0].name;
            }
            else
            {
                command_interface_types_[i] = "velocity";  // 默认使用速度接口
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> FourWheelChassisHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> FourWheelChassisHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            const auto & type = command_interface_types_[i];
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, type, &hw_commands_[i]));
        }
        return command_interfaces;
    }
    
    hardware_interface::CallbackReturn FourWheelChassisHW::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;  // 新增：显式声明参数未使用
        auto cm_node = rclcpp::Node::make_shared("_internal_cm_node");  // 临时节点，仅取接口
        node_base_     = cm_node->get_node_base_interface();
        node_logging_  = cm_node->get_node_logging_interface();
        node_topics_   = cm_node->get_node_topics_interface();
        node_services_ = cm_node->get_node_services_interface();

        state_pub_ = rclcpp::create_publisher<robot_msgs::msg::MotorsState>(
            node_topics_,
            "/ros_robot_controller/set_motor",
            rclcpp::QoS(10)
        );

        RCLCPP_INFO(node_logging_->get_logger(), "FourWheelChassisHW activated, pub & srv ready.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FourWheelChassisHW::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        state_pub_.reset();
        RCLCPP_INFO(node_logging_->get_logger(), "FourWheelChassisHW deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }   

    hardware_interface::return_type FourWheelChassisHW::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type FourWheelChassisHW::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        state_pub_->publish(robot_msgs::msg::MotorsState(
            robot_msgs::msg::MotorsState().set__data({
                robot_msgs::msg::MotorState().set__id(1).set__rps(hw_commands_[0]*10),
                robot_msgs::msg::MotorState().set__id(2).set__rps(-hw_commands_[1]*10),
                robot_msgs::msg::MotorState().set__id(3).set__rps(-hw_commands_[2]*10),
                robot_msgs::msg::MotorState().set__id(4).set__rps(hw_commands_[3]*10),
            })
        ));
        hw_velocities_= hw_commands_;

        hw_positions_  = hw_positions_;
        
        // RCLCPP_INFO(node_logging_->get_logger(), "%s %s %s %s",
        //     std::to_string(hw_commands_[0]).c_str(),
        //     std::to_string(hw_commands_[1]).c_str(),
        //     std::to_string(hw_commands_[2]).c_str(),
        //     std::to_string(hw_commands_[3]).c_str()
        // );

        return hardware_interface::return_type::OK;
    }

}  // namespace four_wheel_chassis_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(four_wheel_chassis_hw::FourWheelChassisHW, hardware_interface::SystemInterface)