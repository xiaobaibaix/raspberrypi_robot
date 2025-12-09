#include "mecanum_wheel_chassis_hw/mecanum_wheel_chassis_hw.hpp"
#include <vector>
#include <string>

namespace mecanum_wheel_chassis_hw
{
    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_init(
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

    std::vector<hardware_interface::StateInterface> MecanumWheelChassisHW::export_state_interfaces()
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

    std::vector<hardware_interface::CommandInterface> MecanumWheelChassisHW::export_command_interfaces()
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
    
    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_activate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        motor_driver_ = new MecanumMotorDriver("/dev/ttyUSB0", 115200, std::chrono::milliseconds(50));
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        delete motor_driver_;
        motor_driver_ = nullptr;
        return hardware_interface::CallbackReturn::SUCCESS;
    }   

    hardware_interface::return_type MecanumWheelChassisHW::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        // 读取编码器值
        auto encoders = motor_driver_->readEncoder();
        for (size_t i = 0; i < hw_positions_.size(); ++i)
        {   
            // 假设每转一圈编码器计数为 4096
            double position = static_cast<double>(encoders[i]) / 4096.0 * 2.0 * 3.141592653589793;
            hw_velocities_[i] = (position - hw_positions_[i]) / period.seconds();
            hw_positions_[i] = position;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MecanumWheelChassisHW::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        // 将速度命令转换为 PWM 信号
        std::array<int16_t, 4> pwm_commands;
        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            // 简单线性映射，假设最大速度对应最大 PWM 值    
            pwm_commands[i] = static_cast<int16_t>(hw_commands_[i] / 10.0 * 255.0);
            if (pwm_commands[i] > 255) pwm_commands[i] = 255;
            if (pwm_commands[i] < -255) pwm_commands[i] = -255;
        }
        motor_driver_->writeSpeed(pwm_commands);
        return hardware_interface::return_type::OK;
    }

}  // namespace four_wheel_chassis_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_wheel_chassis_hw::MecanumWheelChassisHW, hardware_interface::SystemInterface)