#include <algorithm>
#include <chrono>

#include "ferrari_control/controller.hpp"

Controller::Controller(const std::string &cmd_pub_name, const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode(node_name, options),
      cmd_pub_name_(cmd_pub_name)
{
    max_steering_angle_ = this->declare_parameter<double>("max_steering_angle", 0.3448);
    max_speed_ = this->declare_parameter<double>("max_speed", 1.0);
    max_lateral_acceleration_ = this->declare_parameter<double>("max_lateral_acceleration", 3.0);
    max_longitudinal_acceleration_ = this->declare_parameter<double>("max_longitudinal_acceleration", 8.0);
    max_longitudinal_deceleration_ = this->declare_parameter<double>("max_longitudinal_deceleration", 12.0);
    wheelbase_ = this->declare_parameter<double>("wheelbase", 0.218);
    track_width_ = this->declare_parameter<double>("track_width", 0.135);
    wheel_diameter_ = this->declare_parameter<double>("wheel_diameter", 0.0575);
    wheel_thickness_ = this->declare_parameter<double>("wheel_thickness", 0.025);

    double control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 50.0);
    control_period_ = std::chrono::duration<double>(1.0 / control_rate_hz_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/odom", 10, std::bind(&Controller::odomCallback, this, std::placeholders::_1));
    local_trajectory_sub_ = this->create_subscription<ferrari_planning::msg::Trajectory>("/planning/local_trajectory", 10, std::bind(&Controller::localTrajectoryCallback, this, std::placeholders::_1));
}

void Controller::publishCmd(ackermann_msgs::msg::AckermannDriveStamped msg)
{
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    cmd_pub_->publish(msg);
}

void Controller::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = *msg;
}

void Controller::localTrajectoryCallback(const ferrari_planning::msg::Trajectory::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(local_trajectory_mutex_);
    local_trajectory_ = *msg;
}

LifecycleNodeInterface::CallbackReturn Controller::on_configure(const rclcpp_lifecycle::State &)
{
    if (!param_callback_handle_)
    {
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Controller::onParamsChanged, this, std::placeholders::_1));
    }

    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(cmd_pub_name_, 10);
    control_timer_ = this->create_wall_timer(control_period_, std::bind(&Controller::computeControlCommand, this));
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn Controller::on_activate(const rclcpp_lifecycle::State &)
{
    cmd_pub_->on_activate();
    is_active_ = true;
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn Controller::on_deactivate(const rclcpp_lifecycle::State &)
{
    is_active_ = false;
    cmd_pub_->on_deactivate();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn Controller::on_cleanup(const rclcpp_lifecycle::State &)
{
    cmd_pub_.reset();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn Controller::on_shutdown(const rclcpp_lifecycle::State &)
{
    cmd_pub_.reset();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult Controller::onParamsChanged(const std::vector<rclcpp::Parameter> &parameters)
{
    (void)parameters;

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "Parameter change not handled in base Controller class.";
    return result;
}
