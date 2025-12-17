#include "position_tracking_controller/position_tracking_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include <tf2/utils.h>

namespace position_tracking_controller
{
    controller_interface::CallbackReturn PositionTrackingController::on_init()
    {
        // 必须先调父类，否则 reference_interfaces_ 没初始化
        // auto ret = controller_interface::ChainableControllerInterface::on_init();
        // if (ret != CallbackReturn::SUCCESS)
        //     return ret;

        // 为即将导出的 3 个 reference 接口预留内存
        reference_interfaces_.resize(3);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PositionTrackingController::command_interface_configuration() const
    {

        return {
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {"mecanum_x_position_pid/mecanum_drive_controller/linear/x/position",
             "mecanum_y_position_pid/mecanum_drive_controller/linear/y/position",
             "mecanum_z_angle_pid/mecanum_drive_controller/angular/z/position"}};
    }

    controller_interface::InterfaceConfiguration PositionTrackingController::state_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::ALL, {}};
    }

    controller_interface::CallbackReturn PositionTrackingController::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {

        // 初始目标设为 0
        geometry_msgs::msg::Pose2D p;
        p.x = p.y = p.theta = 0.0;
        current_pose_.writeFromNonRT(p);
        target_pose_.writeFromNonRT(p);

        // Action server
        action_server_ = rclcpp_action::create_server<ControllerReferenceMsg>(
            get_node(), "to_pose",
            std::bind(&PositionTrackingController::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&PositionTrackingController::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&PositionTrackingController::handle_accepted, this,
                      std::placeholders::_1));

        // Odom 订阅
        odom_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SystemDefaultsQoS(),
            std::bind(&PositionTrackingController::odom_callback, this,
                      std::placeholders::_1));

        feedback_x_pub_= get_node()->create_publisher<ControllerFeedbackMsg>(
            "mecanum_x_position_pid/measured_state", rclcpp::SystemDefaultsQoS());
        feedback_y_pub_= get_node()->create_publisher<ControllerFeedbackMsg>(
            "mecanum_y_position_pid/measured_state", rclcpp::SystemDefaultsQoS());
        feedback_z_pub_= get_node()->create_publisher<ControllerFeedbackMsg>(
            "mecanum_z_angle_pid/measured_state", rclcpp::SystemDefaultsQoS());
                            
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionTrackingController::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionTrackingController::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PositionTrackingController::update_reference_from_subscribers(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // 把“目标位姿”写到 reference 接口
        auto tgt = target_pose_.readFromRT();
        ref_x_ = tgt->x;
        ref_y_ = tgt->y;
        ref_theta_ = tgt->theta;
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type PositionTrackingController::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // 把“当前位姿”写到 reference 接口，供下游 PID 读取做差
        auto cur = current_pose_.readFromRT();
        ref_x_ = cur->x; // 注意：这里不会冲突，下游 PID 读的是另外 3 个接口
        ref_y_ = cur->y;
        ref_theta_ = cur->theta;
        return controller_interface::return_type::OK;
    }


    std::vector<hardware_interface::CommandInterface> 
    PositionTrackingController::on_export_reference_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> cmd_ifs;
        cmd_ifs.emplace_back(
            get_node()->get_name(), "linear/x/position", &ref_x_);
        cmd_ifs.emplace_back(
            get_node()->get_name(), "linear/y/position", &ref_y_);
        cmd_ifs.emplace_back(
            get_node()->get_name(), "angular/z/position", &ref_theta_);
        return cmd_ifs;
    }

    // ---------------- 以下常规实现 ----------------
    void PositionTrackingController::odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::Pose2D p;
        p.x = msg->pose.pose.position.x;
        p.y = msg->pose.pose.position.y;
        p.theta = tf2::getYaw(msg->pose.pose.orientation);
        // 发布反馈
        ControllerFeedbackMsg fb_msg;
        fb_msg.dof_names.resize(1);
        fb_msg.values.resize(1);
        fb_msg.values_dot.resize(1);

        fb_msg.set__values_dot({0.0}); // 必须初始化，否则发布会报错

        fb_msg.dof_names[0] = "mecanum_drive_controller/linear/x";
        fb_msg.values[0] = p.x;
        feedback_x_pub_->publish(fb_msg);
        fb_msg.dof_names[0] = "mecanum_drive_controller/linear/y";
        fb_msg.values[0] = p.y;
        feedback_y_pub_->publish(fb_msg);
        fb_msg.dof_names[0] = "mecanum_drive_controller/angular/z";
        fb_msg.values[0] = p.theta;
        feedback_z_pub_->publish(fb_msg);
        // 写实时缓存
        current_pose_.writeFromNonRT(p);
    }

    rclcpp_action::GoalResponse
    PositionTrackingController::handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ControllerReferenceMsg::Goal> goal)
    {
        // 简单接受任何目标
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    PositionTrackingController::handle_cancel(
        const std::shared_ptr<GoalHandle>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void
    PositionTrackingController::handle_accepted(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        // 直接覆盖目标
        auto goal = goal_handle->get_goal();
        geometry_msgs::msg::Pose2D p;
        p.x = goal->target_pose.pose.position.x;
        p.y = goal->target_pose.pose.position.y;
        p.theta = tf2::getYaw(goal->target_pose.pose.orientation);
        target_pose_.writeFromNonRT(p);
    }

} // namespace position_tracking_controller

PLUGINLIB_EXPORT_CLASS(
    position_tracking_controller::PositionTrackingController,
    controller_interface::ChainableControllerInterface)