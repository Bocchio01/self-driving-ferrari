#include <chrono>
#include <mutex>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ferrari_planning/msg/trajectory.hpp>

#include "ferrari_control/msg/tracking_error.hpp"
#include "ferrari_control/trajectory_utils.hpp"

class TrackingMonitorNode : public rclcpp::Node
{
public:
    TrackingMonitorNode() : Node("tracking_monitor_node")
    {
        const double rate_hz = this->declare_parameter<double>("rate_hz", 30.0);
        input_timeout_s_ = this->declare_parameter<double>("input_timeout_s", 0.5);

        error_pub_ = this->create_publisher<ferrari_control::msg::TrackingError>("tracking_error", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                       {
            odom_ = *msg;
            last_odom_time_ = this->now(); });
        trajectory_sub_ = this->create_subscription<ferrari_planning::msg::Trajectory>("local_trajectory", 10, [this](const ferrari_planning::msg::Trajectory::SharedPtr msg)
                                                                                       {
            trajectory_ = *msg;
            last_trajectory_time_ = this->now(); });

        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), std::bind(&TrackingMonitorNode::publishError, this));
    }

private:
    void publishError()
    {
        const auto now = this->now();
        const auto timeout = rclcpp::Duration::from_seconds(input_timeout_s_);
        if (last_odom_time_.nanoseconds() == 0 || last_trajectory_time_.nanoseconds() == 0 ||
            now - last_odom_time_ > timeout || now - last_trajectory_time_ > timeout || trajectory_.points.empty())
        {
            return;
        }

        const auto horizon = ferrari_control::trajectory_utils::getTrajectoryHorizonBySpace(odom_.pose.pose, trajectory_, 2);
        const auto error = ferrari_control::trajectory_utils::computeFrenetError(odom_.pose.pose, horizon);

        ferrari_control::msg::TrackingError msg;
        msg.header.stamp = now;
        msg.header.frame_id = odom_.header.frame_id;
        msg.longitudinal_error = error.s;
        msg.lateral_error = error.d;
        msg.heading_error = error.mu;
        msg.target_speed = ferrari_control::trajectory_utils::getTargetVelocity(trajectory_, odom_.pose.pose);
        msg.speed = odom_.twist.twist.linear.x;
        error_pub_->publish(msg);
    }

    double input_timeout_s_ = 0.5;
    nav_msgs::msg::Odometry odom_;
    ferrari_planning::msg::Trajectory trajectory_;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<ferrari_control::msg::TrackingError>::SharedPtr error_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ferrari_planning::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
