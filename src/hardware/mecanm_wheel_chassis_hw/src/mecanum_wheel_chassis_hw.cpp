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
        }

        serial_port_ = "/dev/ttyUSB0"; // 默认值
        baud_rate_ = 115200;           // 默认值

        if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end())
        {
            serial_port_ = info_.hardware_parameters.at("serial_port");
        }

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

        max_pwm_ = 255.0; // 最大PWM值
        if (info_.hardware_parameters.find("max_pwm") != info_.hardware_parameters.end())
        {
            try
            {
                max_pwm_ = std::stod(info_.hardware_parameters.at("max_pwm"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "max_pwm 参数转换失败: %s", e.what());
            }
        }
        min_pwm_ = 0; // 最小PWM值
        if (info_.hardware_parameters.find("min_pwm") != info_.hardware_parameters.end())
        {
            try
            {
                min_pwm_ = std::stod(info_.hardware_parameters.at("min_pwm"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "min_pwm 参数转换失败: %s", e.what());
            }
        }

        max_speed_ = {10, 10, 10, 10}; // 默认最大速度10m/s
        if (info_.hardware_parameters.find("lf_max_speed") != info_.hardware_parameters.end())
        {
            try
            {
                max_speed_[0] = std::stod(info_.hardware_parameters.at("lf_max_speed"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "lf_max_speed 参数转换失败: %s", e.what());
            }
        }
        if (info_.hardware_parameters.find("rf_max_speed") != info_.hardware_parameters.end())
        {
            try
            {
                max_speed_[1] = std::stod(info_.hardware_parameters.at("rf_max_speed"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "rf_max_speed 参数转换失败: %s", e.what());
            }
        }
        if (info_.hardware_parameters.find("lr_max_speed") != info_.hardware_parameters.end())
        {
            try
            {
                max_speed_[2] = std::stod(info_.hardware_parameters.at("lr_max_speed"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "lr_max_speed 参数转换失败: %s", e.what());
            }
        }
        if (info_.hardware_parameters.find("rr_max_speed") != info_.hardware_parameters.end())
        {
            try
            {
                max_speed_[3] = std::stod(info_.hardware_parameters.at("rr_max_speed"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"),
                             "rr_max_speed 参数转换失败: %s", e.what());
            }
        }
        // 在 on_init 里
        auto logger = rclcpp::get_logger("MecanumWheelChassisHW");

        auto get_bool_param = [&](const std::string &key) -> bool
        {
            std::string lower;
            lower.resize(key.size());
            std::transform(key.begin(), key.end(), lower.begin(), ::tolower);
            return lower == "true" || lower == "1" || lower == "yes";
        };

        auto it = info_.hardware_parameters.find("use_topic");
        if (it != info_.hardware_parameters.end())
            use_topic_ = get_bool_param(it->second);

        auto it2 = info_.hardware_parameters.find("use_server");
        if (it2 != info_.hardware_parameters.end())
            use_server_ = get_bool_param(it2->second);

        RCLCPP_INFO(logger, "use_topic: %s", use_topic_ ? "true" : "false");
        RCLCPP_INFO(logger, "use_server: %s", use_server_ ? "true" : "false");

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

        if (use_topic_ || use_server_)
        {
            motor_driver_ = nullptr;
            // 创建节点
            node_ = std::make_shared<rclcpp::Node>("mecanum_wheel_chassis_hw_node");

            // 订阅 /cmd_vel
            cmd_pub_ = node_->create_publisher<robot_msgs::msg::MotorsState>("/ros_robot_controller/set_motor", 10);
            if (use_topic_)
            {
                pos_sub_ = node_->create_subscription<robot_msgs::msg::PWMServoState>("/ros_robot_controller/motor_pos", 10,
                                                                                      [&](const robot_msgs::msg::PWMServoState::SharedPtr msg)
                                                                                      {
                                                                                          std::lock_guard<std::mutex> lock(sb_queue_mutex_);
                                                                                          for (size_t i = 0; i < msg->id.size(); ++i)
                                                                                          {
                                                                                              size_t idx = msg->id[i] - 1; // 先检查越界更稳妥
                                                                                              if (idx < sub_queues_.size())
                                                                                                  sub_queues_[idx].push_back({msg->id[i], msg->position[i]});
                                                                                          }
                                                                                      });
                for (auto &q : sub_queues_)
                    q.set_capacity(Q_LEN);
                RCLCPP_INFO(logger, "created sub client for PWMServoState");
            }
            if (use_server_)
            {
                pos_cli_ = node_->create_client<robot_msgs::srv::GetPWMServoState>("/ros_robot_controller/pwm_servo/get_state");
                // 创建服务客户端
                RCLCPP_INFO(logger, "created service client for GetPWMServoState");
            }
        }
        else
        {
            node_ = nullptr;
            cmd_pub_ = nullptr;
            pos_sub_ = nullptr;
            pos_cli_ = nullptr;
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
        if (use_topic_ || use_server_)
        {
            if (use_server_)
            {
                while (!pos_cli_->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                        return hardware_interface::CallbackReturn::ERROR;
                    RCLCPP_WARN(node_->get_logger(), "waiting for service ...");
                }
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("MecanumWheelChassisHW"),
                        "Not using topic or service, using MecanumMotorDriver directly.");
            // 直接使用 MecanumMotorDriver
            motor_driver_ = new MecanumMotorDriver(serial_port_, baud_rate_);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        if (nullptr != motor_driver_)
        {
            delete motor_driver_;
            motor_driver_ = nullptr;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MecanumWheelChassisHW::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        if (use_topic_ && use_server_)
        {
            // 请求
            auto req = std::make_shared<robot_msgs::srv::GetPWMServoState::Request>();
            robot_msgs::msg::GetPWMServoCmd m1;
            m1.id = 1;           // uint8
            m1.get_position = 1; // 获取位置
            m1.get_offset = 0;   // 不获取偏移
            req->cmd.push_back(m1);
            robot_msgs::msg::GetPWMServoCmd m2;
            m2.id = 2;           // uint8
            m2.get_position = 1; // 获取位置
            m2.get_offset = 0;   // 不获取偏移
            req->cmd.push_back(m2);
            robot_msgs::msg::GetPWMServoCmd m3;
            m3.id = 3;           // uint8
            m3.get_position = 1; // 获取位置
            m3.get_offset = 0;   // 不获取偏移
            req->cmd.push_back(m3);
            robot_msgs::msg::GetPWMServoCmd m4;
            m4.id = 4;           // uint8
            m4.get_position = 1; // 获取位置
            m4.get_offset = 0;   // 不获取偏移
            req->cmd.push_back(m4);
            auto future = pos_cli_->async_send_request(req);

            if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(10)) == rclcpp::FutureReturnCode::SUCCESS)
            {
                timeout_cnt_ = 0; // 收到包就清零
                auto resp = future.get();
                if (resp->state.size() != 4)
                { // 直接判长度
                    RCLCPP_ERROR(this->get_logger(), "Motor data size != 4");
                    for(auto &s : resp->state)
                    {
                        RCLCPP_ERROR(this->get_logger(), "id:%d pos size:%zu", s.id, s.position.size());
                    }
                    return hardware_interface::return_type::OK;
                }

                for (size_t i = 0; i < 4; ++i)
                {
                    if (resp->state[i].position.empty())
                    {
                        RCLCPP_ERROR(this->get_logger(), "motor %zu position empty", i);
                        continue;
                    }

                    /* 1. 原始角度 */
                    double raw_angle = (resp->state[i].position.front() / encoder_ppr_) * 2.0 * M_PI / gear_ratio_;
                    /* 2. 低通滤波速度 */
                    double raw_v = (raw_angle - hw_positions_[i]) / period.seconds();
                    hw_velocities_[i] = (1.0 - VELOCITY_ALPHA) * hw_velocities_[i] + VELOCITY_ALPHA * raw_v;
                    /* 3. 更新位置 */
                    hw_positions_[i] = raw_angle;
                }
            }
            else
            { 
                // 超时分支 —— 匀速模型
                ++timeout_cnt_;
                if (timeout_cnt_ > MAX_TIMEOUT)
                { // 长期丢包 → 停车
                    for (size_t i = 0; i < 4; ++i)
                        hw_velocities_[i] = 0.0;
                    timeout_cnt_ = 0;
                }
                for (size_t i = 0; i < 4; ++i)
                {
                    /* 匀速外推：速度不变，位置积分一步 */
                    hw_positions_[i] += hw_velocities_[i] * period.seconds();
                    // 速度保持上一周期值，无需再赋值
                }
                RCLCPP_INFO(this->get_logger(), "timer out!");

            }
        }
        else if (use_topic_)
        {
            // 直接读取数据并转换
            MotorPosition pos[4];
            {
                std::lock_guard<std::mutex> lock(sb_queue_mutex_);
                for (auto &q : sub_queues_)
                {
                    MotorPosition p = q.back();
                    pos[p.id - 1] = p;
                    q.pop_back();
                }
            }
            for (size_t i = 0; i < hw_positions_.size(); ++i)
            {
                double wheel_angle = (pos[i].encode / encoder_ppr_) * 2.0 * M_PI / gear_ratio_;
                hw_velocities_[i] = (wheel_angle - hw_positions_[i]) / period.seconds();
                hw_positions_[i] = wheel_angle;
            }
        }
        else
        {
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
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MecanumWheelChassisHW::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        if (use_topic_)
        {
            // 限制输出30以下没有输出
            double tmp_commands_[4];
            for (size_t i = 0; i < hw_commands_.size(); ++i)
            {
                // if (std::abs(hw_commands_[i]) <= 40)
                // {
                //     tmp_commands_[i] = 0.0;
                // }
                // else
                    tmp_commands_[i] = hw_commands_[i];
            }

            // topic 发送指令
            cmd_pub_->publish(robot_msgs::msg::MotorsState(
                robot_msgs::msg::MotorsState().set__data({
                    robot_msgs::msg::MotorState().set__id(1).set__rps(static_cast<int16_t>(tmp_commands_[0])),
                    robot_msgs::msg::MotorState().set__id(2).set__rps(static_cast<int16_t>(tmp_commands_[1])),
                    robot_msgs::msg::MotorState().set__id(3).set__rps(static_cast<int16_t>(tmp_commands_[2])),
                    robot_msgs::msg::MotorState().set__id(4).set__rps(static_cast<int16_t>(tmp_commands_[3])),
                })));
            // RCLCPP_INFO(rclcpp::get_logger("MecanumWheelChassisHW"),
            //     "Published motor commands via topic: [%f, %f, %f, %f]",
            //     hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3]);
        }
        else
        {
            // 将速度命令转换为 PWM 信号
            std::array<int16_t, 4> pwm_commands = {0};
            for (size_t i = 0; i < hw_commands_.size(); ++i)
            {
                pwm_commands[i] = static_cast<int16_t>(hw_commands_[i]); // 简单线性映射，实际应用中可能需要更复杂的转换
            }
            motor_driver_->writeSpeed(pwm_commands);
        }
        return hardware_interface::return_type::OK;
    }

} // namespace four_wheel_chassis_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_wheel_chassis_hw::MecanumWheelChassisHW, hardware_interface::SystemInterface)