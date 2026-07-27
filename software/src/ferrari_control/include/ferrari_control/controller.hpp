#pragma once

#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <ferrari_vehicle/vehicle_model_factory.hpp>
#include <ferrari_planning/msg/trajectory.hpp>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class Controller : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit Controller(
        const std::string &cmd_pub_name_,
        const std::string &node_name,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    void publishCmd(ackermann_msgs::msg::AckermannDriveStamped msg);
    virtual void computeControlCommand() = 0;

    // Lifecycle transition hooks
    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
    LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
    virtual rcl_interfaces::msg::SetParametersResult onParamsChanged(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ferrari_planning::msg::Trajectory>::SharedPtr local_trajectory_sub_;
    std::chrono::duration<double> control_period_;

    bool is_active_ = false;

    // Command publisher name
    std::string cmd_pub_name_;

    // Local path and odometry data
    std::mutex local_trajectory_mutex_;
    ferrari_planning::msg::Trajectory local_trajectory_;
    nav_msgs::msg::Odometry odom_;

    // Vehicle parameters
    double max_steering_angle_ = 0.3448;
    double max_speed_ = 1.0;
    double max_lateral_acceleration_ = 3.0;
    double max_longitudinal_acceleration_ = 8.0;
    double max_longitudinal_deceleration_ = 12.0;
    double wheelbase_ = 0.218;
    double track_width_ = 0.135;
    double wheel_diameter_ = 0.0575;
    double wheel_thickness_ = 0.025;

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void localTrajectoryCallback(const ferrari_planning::msg::Trajectory::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr control_timer_;
};