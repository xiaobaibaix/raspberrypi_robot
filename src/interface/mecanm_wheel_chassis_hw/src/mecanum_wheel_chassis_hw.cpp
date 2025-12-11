#include "mecanum_wheel_chassis_hw/mecanum_wheel_chassis_hw.hpp"
#include <vector>
#include <string>

namespace mecanum_wheel_chassis_hw
{
    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("MecanumWheelChassisHW"), 
                    "没有硬件参数，使用默认值");
        }
        
        std::string serial_port = "/dev/ttyUSB0";  // 默认值
        uint32_t baud_rate = 115200;               // 默认值
        
        if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end()) {
            serial_port = info_.hardware_parameters.at("serial_port");
        }
        
        if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
            try {
                baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "baud_rate 参数转换失败: %s", e.what());
            }
        }
        
        encoder_ppr_ = 4096.0;  // 编码器每转脉冲数
        if (info_.hardware_parameters.find("encoder_ppr") != info_.hardware_parameters.end()) {
            try {
                encoder_ppr_ = std::stod(info_.hardware_parameters.at("encoder_ppr"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "encoder_ppr 参数转换失败: %s", e.what());
            }
        }
        
        wheel_radius_ = 0.05;  // 轮子半径 (米)
        if (info_.hardware_parameters.find("wheel_radius") != info_.hardware_parameters.end()) {
            try {
                wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "wheel_radius 参数转换失败: %s", e.what());
            }
        }
        
        gear_ratio_ = 1.0;  // 减速比
        if (info_.hardware_parameters.find("gear_ratio") != info_.hardware_parameters.end()) {
            try {
                gear_ratio_ = std::stod(info_.hardware_parameters.at("gear_ratio"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "gear_ratio 参数转换失败: %s", e.what());
            }
        }
        
        max_pwm_ = 255;  // 最大PWM值
        if (info_.hardware_parameters.find("max_pwm") != info_.hardware_parameters.end()) {
            try {
                max_pwm_ = std::stoi(info_.hardware_parameters.at("max_pwm"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "max_pwm 参数转换失败: %s", e.what());
            }
        }
        min_pwm_ = -255;  // 最小PWM值
        if (info_.hardware_parameters.find("min_pwm") != info_.hardware_parameters.end()) {
            try {
                max_pwm_ = std::stoi(info_.hardware_parameters.at("min_pwm"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "max_pwm 参数转换失败: %s", e.what());
            }
        }

        max_speed_={10,10,10,10}; // 默认最大速度10m/s
        if (info_.hardware_parameters.find("lf_max_speed") != info_.hardware_parameters.end()) {
            try {
                max_speed_[0] = std::stod(info_.hardware_parameters.at("lf_max_speed"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "lf_max_speed 参数转换失败: %s", e.what());
            }
        }       
        if (info_.hardware_parameters.find("rf_max_speed") != info_.hardware_parameters.end()) {
            try {
                max_speed_[1] = std::stod(info_.hardware_parameters.at("rf_max_speed"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "rf_max_speed 参数转换失败: %s", e.what());
            }
        }     
        if (info_.hardware_parameters.find("lr_max_speed") != info_.hardware_parameters.end()) {
            try {
                max_speed_[2] = std::stod(info_.hardware_parameters.at("lr_max_speed"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "lr_max_speed 参数转换失败: %s", e.what());
            }
        }     
        if (info_.hardware_parameters.find("rr_max_speed") != info_.hardware_parameters.end()) {
            try {
                max_speed_[3] = std::stod(info_.hardware_parameters.at("rr_max_speed"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "rr_max_speed 参数转换失败: %s", e.what());
            }
        }     
        serial_port_ = serial_port;
        baud_rate_ = baud_rate;

        use_topic_=false;
        use_server_=false;

        if (info_.hardware_parameters.find("use_topic") != info_.hardware_parameters.end()) {
            try {
                max_speed_[2] = std::stod(info_.hardware_parameters.at("use_topic"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "use_topic 参数转换失败: %s", e.what());
            }
        }     
        if (info_.hardware_parameters.find("use_server") != info_.hardware_parameters.end()) {
            try {
                max_speed_[3] = std::stod(info_.hardware_parameters.at("use_server"));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("MecanumWheelChassisHW"), 
                            "use_server 参数转换失败: %s", e.what());
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
                command_interface_types_[i] = "velocity";  // 默认使用速度接口
            }
        }

        if(use_topic_||use_server_){
            motor_driver_=nullptr;
            // 创建节点
            node_ = std::make_shared<rclcpp::Node>("mecanum_wheel_chassis_hw_node");

            // 订阅 /cmd_vel
            cmd_pub_ = node_->create_publisher<robot_msgs::msg::MotorsState>("/ros_robot_controller/set_motor", 10);
            if(use_topic_){
                pos_sub_ = node_->create_subscription<robot_msgs::msg::PWMServoState>("/ros_robot_controller/motor_pos", 10,
                [&](const robot_msgs::msg::PWMServoState::SharedPtr msg){
                    std::lock_guard<std::mutex> lock(sb_queue_mutex_);
                    for (size_t i = 0; i < msg->id.size(); ++i)
                    {
                        size_t idx = msg->id[i] - 1;          // 先检查越界更稳妥
                        if (idx < sub_queues_.size())
                            sub_queues_[idx].push_back({msg->id[i],msg->position[i]});
                    }
                });
                for (auto &q : sub_queues_) q.set_capacity(Q_LEN);
            }
            if(use_server_)pos_cli_ = node_->create_client<robot_msgs::srv::GetPWMServoState>("/ros_robot_controller/get_motor_pos");

        }else{
            node_=nullptr;
            cmd_pub_=nullptr;
            pos_sub_=nullptr;
            pos_cli_=nullptr;
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
        if(use_server_){
            while (!pos_cli_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) return hardware_interface::CallbackReturn::ERROR;
                RCLCPP_WARN(node_->get_logger(), "waiting for service ...");
            }
        }else{
            motor_driver_ = new MecanumMotorDriver(serial_port_, baud_rate_);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }   

    hardware_interface::CallbackReturn MecanumWheelChassisHW::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        if(nullptr!=motor_driver_){
            delete motor_driver_;
            motor_driver_ = nullptr;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }   

    hardware_interface::return_type MecanumWheelChassisHW::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        if(use_topic_&&use_server_){
            //请求
            auto req = std::make_shared<robot_msgs::srv::GetPWMServoState::Request>();
            robot_msgs::msg::GetPWMServoCmd m1;
            m1.id            = 1;          // uint8
            req->cmd.push_back(m1);
            robot_msgs::msg::GetPWMServoCmd m2;
            m2.id            = 2;          // uint8
            req->cmd.push_back(m2);
            robot_msgs::msg::GetPWMServoCmd m3;
            m3.id            = 3;          // uint8
            req->cmd.push_back(m3);
            robot_msgs::msg::GetPWMServoCmd m4;
            m4.id            = 4;          // uint8
            req->cmd.push_back(m4);
            auto future = pos_cli_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2))
                == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto resp = future.get();
                // 遍历返回的 state 数组
                for (size_t i = 0; i < hw_positions_.size(); ++i)
                {
                    double wheel_angle = (resp->state[i].position.front() / encoder_ppr_) * 2.0 * M_PI / gear_ratio_;
                    hw_velocities_[i] = (wheel_angle - hw_positions_[i]) / period.seconds();
                    hw_positions_[i] = wheel_angle;
                }
            }
        }else if(use_topic_){
            //直接读取数据并转换
            MotorPosition pos[4];
            {
                std::lock_guard<std::mutex> lock(sb_queue_mutex_);
                for(auto &q: sub_queues_){
                    MotorPosition p = q.back();
                    pos[p.id-1] = p;
                    q.pop_back();
                }
            }
            for (size_t i = 0; i < hw_positions_.size(); ++i)
            {
                double wheel_angle = (pos[i].encode / encoder_ppr_) * 2.0 * M_PI / gear_ratio_;
                hw_velocities_[i] = (wheel_angle - hw_positions_[i]) / period.seconds();
                hw_positions_[i] = wheel_angle;
            }
        }else{
            // 读取编码器值
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
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        if(use_topic_||use_server_){
            //topic 发送指令

        }else{
            // 将速度命令转换为 PWM 信号
            std::array<int16_t, 4> pwm_commands={0};
            for (size_t i = 0; i < hw_commands_.size(); ++i)
            {
                pwm_commands[i] = static_cast<int16_t>(hw_commands_[i]*10); // 简单线性映射，实际应用中可能需要更复杂的转换
                // RCLCPP_INFO(rclcpp::get_logger("MecanumWheelChassisHW"), 
                // "cmd:%f pwm:%d",hw_commands_[i],pwm_commands[i]);
                // if (pwm_commands[i] > max_pwm_) pwm_commands[i] = max_pwm_;
                // if (pwm_commands[i] < min_pwm_) pwm_commands[i] = min_pwm_;
                // RCLCPP_INFO(rclcpp::get_logger("MecanumWheelChassisHW"), 
                // "cmd:%f pwm:%d",hw_commands_[i],pwm_commands[i]);
            }
            // motor_driver_->writeSpeed(pwm_commands);
        }
        return hardware_interface::return_type::OK;
    }

}  // namespace four_wheel_chassis_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_wheel_chassis_hw::MecanumWheelChassisHW, hardware_interface::SystemInterface)