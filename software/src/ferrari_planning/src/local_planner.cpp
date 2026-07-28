#include <algorithm>
#include <chrono>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "ferrari_planning/local_planner.hpp"

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions &options)
    : Node("local_planner_node", options),
      closest_path_index_(0),
      obstacles_detected_(false)
{
    this->lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 2.5);
    this->point_spacing_ = this->declare_parameter<double>("point_spacing", 0.1);
    double update_rate_hz = this->declare_parameter<double>("update_rate_hz", 10.0);

    this->local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);
    this->global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("global_path", 10, std::bind(&LocalPlannerNode::globalPathCallback, this, std::placeholders::_1));
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&LocalPlannerNode::odomCallback, this, std::placeholders::_1));
    this->lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("lidar", 10, std::bind(&LocalPlannerNode::lidarCallback, this, std::placeholders::_1));

    auto timer_period = std::chrono::duration<double>(1.0 / update_rate_hz);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&LocalPlannerNode::publishLocalPath, this));
}

void LocalPlannerNode::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    global_path_ = *msg;
    closest_path_index_ = 0;
}

void LocalPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = *msg;
}

void LocalPlannerNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Currently, we are not processing the LiDAR data in this callback. This is a placeholder for future implementation where LiDAR data can be used for obstacle detection or path planning adjustments.
    // Placeholder for DWA/MPC obstacle avoidance logic
    // bool obstacles_detected_ = check_obstacle(msg);
    (void)msg; // Suppress unused variable warning
}

void LocalPlannerNode::publishLocalPath()
{
    if (global_path_.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Global path is empty. Cannot compute local path.");
        return;
    }

    if (obstacles_detected_)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacles detected. Local path may need to be adjusted.");
        local_path_ = computeLocalPath(lookahead_distance_, point_spacing_);
    }
    else
    {
        local_path_ = getLocalPath(lookahead_distance_, point_spacing_);
    }

    local_path_pub_->publish(local_path_);
}

void LocalPlannerNode::updateClosestIndex()
{
    if (global_path_.poses.empty())
        return;

    const std::size_t forward_window = 60;
    const std::size_t backward_window = 10;
    const double fallback_distance = 1.0;

    std::size_t search_start = (closest_path_index_ > backward_window) ? (closest_path_index_ - backward_window) : 0;
    std::size_t search_end = std::min(global_path_.poses.size(), closest_path_index_ + forward_window);

    double min_distance = std::numeric_limits<double>::max();
    std::size_t best_index = closest_path_index_;

    // Search only within the local window
    for (std::size_t i = search_start; i < search_end; ++i)
    {
        double distance = std::hypot(global_path_.poses[i].pose.position.x - odom_.pose.pose.position.x,
                                     global_path_.poses[i].pose.position.y - odom_.pose.pose.position.y);
        if (distance < min_distance)
        {
            min_distance = distance;
            best_index = i;
        }
    }

    // Fallback if the car is lost (e.g., spawned elsewhere or window lagged behind)
    if (min_distance > fallback_distance)
    {
        RCLCPP_WARN(this->get_logger(), "Vehicle lost from local window. Triggering global search.");
        min_distance = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < global_path_.poses.size(); ++i)
        {
            double distance = std::hypot(global_path_.poses[i].pose.position.x - odom_.pose.pose.position.x,
                                         global_path_.poses[i].pose.position.y - odom_.pose.pose.position.y);
            if (distance < min_distance)
            {
                min_distance = distance;
                best_index = i;
            }
        }
    }

    closest_path_index_ = best_index;
}

const nav_msgs::msg::Path LocalPlannerNode::getLocalPath(double lookahead_distance, double point_spacing)
{
    local_path_.header = global_path_.header;
    local_path_.header.stamp = this->now();
    local_path_.poses.clear();

    if (global_path_.poses.empty())
    {
        return local_path_;
    }

    updateClosestIndex();

    // Ensure we have a starting point
    local_path_.poses.push_back(global_path_.poses[closest_path_index_]);

    double accumulated_distance = 0.0;
    double next_target_distance = point_spacing;

    // Iterate through the global path segments starting from the closest point
    for (std::size_t i = closest_path_index_; i < global_path_.poses.size() - 1; ++i)
    {
        const auto &current_pose = global_path_.poses[i];
        const auto &next_pose = global_path_.poses[i + 1];

        double segment_distance = std::hypot(next_pose.pose.position.x - current_pose.pose.position.x,
                                             next_pose.pose.position.y - current_pose.pose.position.y);

        // WHILE loop allows us to drop multiple points if one segment is very long
        while (accumulated_distance + segment_distance >= next_target_distance)
        {
            if (next_target_distance > lookahead_distance)
            {
                // We reached the lookahead distance, exit immediately
                return local_path_;
            }

            double ratio = (next_target_distance - accumulated_distance) / segment_distance;

            geometry_msgs::msg::PoseStamped interpolated_pose;
            interpolated_pose.header = current_pose.header;

            // Position interpolation
            interpolated_pose.pose.position.x = current_pose.pose.position.x + ratio * (next_pose.pose.position.x - current_pose.pose.position.x);
            interpolated_pose.pose.position.y = current_pose.pose.position.y + ratio * (next_pose.pose.position.y - current_pose.pose.position.y);

            // Orientation interpolation (SLERP)
            tf2::Quaternion q_start, q_end;
            tf2::fromMsg(current_pose.pose.orientation, q_start);
            tf2::fromMsg(next_pose.pose.orientation, q_end);

            tf2::Quaternion q_interp = q_start.slerp(q_end, ratio);
            interpolated_pose.pose.orientation = tf2::toMsg(q_interp);

            local_path_.poses.push_back(interpolated_pose);

            // Queue up the next distance target
            next_target_distance += point_spacing;
        }

        // Advance to the next segment on the global path
        accumulated_distance += segment_distance;

        // Safety check to exit loop if we've accumulated enough distance
        if (accumulated_distance >= lookahead_distance)
        {
            break;
        }
    }

    return local_path_;
}

const nav_msgs::msg::Path LocalPlannerNode::computeLocalPath(double lookahead_distance, double point_spacing)
{
    // This function is currently a placeholder for future implementations that may involve more complex path computations, such as obstacle avoidance or dynamic path adjustments.
    // For now, it simply calls getLocalPath to retrieve the local path based on the current global path and odometry data.
    return getLocalPath(lookahead_distance, point_spacing);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}