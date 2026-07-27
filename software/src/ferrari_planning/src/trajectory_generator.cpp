#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ferrari_planning/msg/trajectory.hpp>
#include <ferrari_planning/msg/trajectory_point.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

#include "ferrari_planning/trajectory_generator.hpp"

TrajectoryGeneratorNode::TrajectoryGeneratorNode(const rclcpp::NodeOptions &options)
    : Node("trajectory_generator_node", options)
{
    this->max_steering_angle_ = this->declare_parameter("max_steering_angle", 0.3448);                     // radians
    this->max_speed_ = this->declare_parameter("max_speed", 1.0);                                          // m/s
    this->max_lateral_acceleration_ = this->declare_parameter("max_lateral_acceleration", 3.0);            // m/s^2 (lateral grip)
    this->max_longitudinal_acceleration_ = this->declare_parameter("max_longitudinal_acceleration", 8.0);  // m/s^2 (throttle)
    this->max_longitudinal_deceleration_ = this->declare_parameter("max_longitudinal_deceleration", 12.0); // m/s^2 (brakes)

    local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("local_path", 10, std::bind(&TrajectoryGeneratorNode::localPathCallback, this, std::placeholders::_1));
    trajectory_pub_ = this->create_publisher<ferrari_planning::msg::Trajectory>("local_trajectory", 10);
}

void TrajectoryGeneratorNode::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Received local path with less than 2 points. Cannot generate trajectory.");
        return;
    }

    ferrari_planning::msg::Trajectory trajectory;
    trajectory.header = msg->header;
    trajectory.header.stamp = this->now();

    std::size_t n = msg->poses.size();
    std::vector<double> distances(n, 0.0);
    std::vector<double> curvatures(n, 0.0);
    std::vector<double> velocities(n, max_speed_);

    // Compute Distances and Curvature Limits
    for (std::size_t i = 1; i < n; ++i)
    {
        double dx = msg->poses[i].pose.position.x - msg->poses[i - 1].pose.position.x;
        double dy = msg->poses[i].pose.position.y - msg->poses[i - 1].pose.position.y;
        distances[i] = std::hypot(dx, dy);

        double yaw_current = tf2::getYaw(msg->poses[i].pose.orientation);
        double yaw_prev = tf2::getYaw(msg->poses[i - 1].pose.orientation);

        // Normalize angle diff to [-pi, pi]
        double d_yaw = std::remainder(yaw_current - yaw_prev, 2.0 * M_PI);

        if (distances[i] > 1e-4)
        {
            curvatures[i] = std::abs(d_yaw) / distances[i];
        }

        // v = sqrt(a_lat / k)
        if (curvatures[i] > 1e-4)
        {
            double v_curve = std::sqrt(max_lateral_acceleration_ / curvatures[i]);
            velocities[i] = std::min(max_speed_, v_curve);
        }
    }

    velocities[0] = velocities[1];

    // Forward Pass (Acceleration Limit)
    for (std::size_t i = 1; i < n; ++i)
    {
        double v_accel = std::sqrt(velocities[i - 1] * velocities[i - 1] + 2.0 * max_longitudinal_acceleration_ * distances[i]);
        velocities[i] = std::min(velocities[i], v_accel);
    }

    // Backward Pass (Deceleration Limit)
    for (int i = n - 2; i >= 0; --i)
    {
        double v_brake = std::sqrt(velocities[i + 1] * velocities[i + 1] + 2.0 * max_longitudinal_deceleration_ * distances[i + 1]);
        velocities[i] = std::min(velocities[i], v_brake);
    }

    // Build Trajectory Message and Compute Timestamps
    double time_accum = 0.0;
    trajectory.points.reserve(n);

    for (std::size_t i = 0; i < n; ++i)
    {
        ferrari_planning::msg::TrajectoryPoint point;
        point.pose = msg->poses[i].pose;

        point.velocity.linear.x = velocities[i];

        // dt = ds / v_avg
        if (i > 0)
        {
            double v_avg = std::max((velocities[i] + velocities[i - 1]) / 2.0, 0.1); // Avoid division by zero
            time_accum += distances[i] / v_avg;
        }

        // Convert double seconds to builtin_interfaces/Duration
        point.time_from_start.sec = static_cast<int32_t>(std::floor(time_accum));
        point.time_from_start.nanosec = static_cast<uint32_t>((time_accum - point.time_from_start.sec) * 1e9);

        // Compute acceleration (dv/dt) for feedforward control
        if (i > 0)
        {
            double dt = distances[i] / std::max((velocities[i] + velocities[i - 1]) / 2.0, 0.1);
            point.acceleration.linear.x = (velocities[i] - velocities[i - 1]) / dt;
        }
        else
        {
            point.acceleration.linear.x = 0.0; // Assume 0 initial accel or compute from telemetry later
        }

        trajectory.points.push_back(point);
    }

    trajectory_pub_->publish(trajectory);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}