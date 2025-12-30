#include "mecanum_wheel_chassis_hw/mecanum_wheel_chassis_hw.hpp"
#include <vector>
#include <string>

namespace mecanum_wheel_chassis_hw
{
    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("MecanumWheelChassisHW"),
                        "没有硬件参数，使用默认值");
            return hardware_interface::CallbackReturn::ERROR;      
        }
        serial_port_="/dev/ttfUSB0";
        if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end())
        {
            serial_port_ = info_.hardware_parameters.at("serial_port");
        }

        baud_rate_=115200;
        if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end())
        {
            try
            {
                baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "baud_rate 参数转换失败: %s", e.what());
            }
        }

        encoder_ppr_ = 4096.0; // 编码器每转脉冲数
        if (info_.hardware_parameters.find("encoder_ppr") != info_.hardware_parameters.end())
        {
            try
            {
                encoder_ppr_ = std::stod(info_.hardware_parameters.at("encoder_ppr"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "encoder_ppr 参数转换失败: %s", e.what());
            }
        }

        wheel_radius_ = 0.05; // 轮子半径 (米)
        if (info_.hardware_parameters.find("wheel_radius") != info_.hardware_parameters.end())
        {
            try
            {
                wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "wheel_radius 参数转换失败: %s", e.what());
            }
        }

        gear_ratio_ = 1.0; // 减速比
        if (info_.hardware_parameters.find("gear_ratio") != info_.hardware_parameters.end())
        {
            try
            {
                gear_ratio_ = std::stod(info_.hardware_parameters.at("gear_ratio"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "gear_ratio 参数转换失败: %s", e.what());
            }
        }

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
                command_interface_types_[i] = "velocity"; // 默认使用速度接口
            }
        }

        motor_driver_ = new MecanumMotorDriver(serial_port_, baud_rate_);

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
            const auto &type = command_interface_types_[i];
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, type, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MecanumWheelChassisHW::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        auto encoders = motor_driver_->readEncoder();
        for (size_t i = 0; i < hw_positions_.size(); ++i)
        {
            // 假设每转一圈编码器计数为 255
            double wheel_angle = (encoders[i] / encoder_ppr_) * 2.0 * M_PI / gear_ratio_;
            hw_velocities_[i] = (wheel_angle - hw_positions_[i]) / period.seconds();
            hw_positions_[i] = wheel_angle;
            // RCLCPP_INFO(rclcpp::get_logger("MecanumWheelChassisHW"),
            // "speed:%f position:%f encode:%d",hw_velocities_[i],hw_positions_[i],encoders[i]);
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MecanumWheelChassisHW::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        static std::array<int16_t, 4> pwm_commands = {0};
        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            pwm_commands[i] = static_cast<int16_t>(hw_commands_[i]); // 简单线性映射，实际应用中可能需要更复杂的转换
        }
        motor_driver_->writeSpeed(pwm_commands);
        return hardware_interface::return_type::OK;
    }

} // namespace four_wheel_chassis_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_wheel_chassis_hw::MecanumWheelChassisHW, hardware_interface::SystemInterface)