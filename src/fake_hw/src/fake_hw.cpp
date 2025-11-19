#include "fake_hw/fake_hw.hpp"

namespace fake_hw
{
    hardware_interface::CallbackReturn FakeHW::on_init(
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
                command_interface_types_[i] = "position";
            }
        }
        
        // // ========== 新增：创建订阅 ==========
        // auto node = rclcpp::Node::make_shared("fake_hw_node");  // 临时节点，生命周期跟着硬件接口
        // wheel_sub_ = node->create_subscription<fake_hw::msg::WheelStates>(
        //     "/wheel_states", 10,
        //     [this](const fake_hw::msg::WheelStates::SharedPtr msg)
        //     { this->on_wheel_states(msg); });

        // // 把 node 存到静态变量里，防止被析构（最简单粗暴办法）
        // static auto static_node = node;  // 生命周期跟随插件库

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> FakeHW::export_state_interfaces()
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

    std::vector<hardware_interface::CommandInterface> FakeHW::export_command_interfaces()
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
    
    hardware_interface::CallbackReturn FakeHW::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FakeHW::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }   

    hardware_interface::return_type FakeHW::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;  // 新增：显式声明参数未使用
        const double dt = period.seconds();
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            const auto & type = command_interface_types_[i];
            if (type == "velocity")
            {
                hw_velocities_[i] = hw_commands_[i];
                hw_positions_[i] += hw_velocities_[i] * dt;  // 简单积分模拟
            }
            else if (type == "position")
            {
                hw_positions_[i] = hw_commands_[i];
                hw_velocities_[i] = 0.0;
            }
            else
            {
                // 其他类型未模拟
                hw_velocities_[i] = 0.0;
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type FakeHW::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;  // 新增：显式声明参数未使用
        (void)period;  // 新增：显式声明参数未使用
        return hardware_interface::return_type::OK;
    }

    // void FakeHW::on_wheel_states(const fake_hw::msg::WheelStates::SharedPtr msg){
    //     const size_t n = std::min(msg->positions.size(), hw_positions_.size());
    //     for (size_t i = 0; i < n; ++i)
    //     {
    //         hw_positions_[i]  = msg->positions[i];
    //         hw_velocities_[i] = msg->velocities[i];
    //     }
    // }
}  // namespace fake_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fake_hw::FakeHW, hardware_interface::SystemInterface)