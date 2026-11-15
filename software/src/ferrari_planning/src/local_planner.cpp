#include <algorithm>
#include <chrono>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "ferrari_planning/local_planner.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions &options)
    : Node("local_planner_node", options),
      closest_path_index_(0),
      obstacles_detected_(false)
{
    this->lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 2.5);
    this->point_spacing_ = this->declare_parameter<double>("point_spacing", 0.1);
    double update_rate_hz = this->declare_parameter<double>("update_rate_hz", 10.0);

    this->local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);
    this->global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("global_path", rclcpp::QoS(1).transient_local(), std::bind(&LocalPlannerNode::globalPathCallback, this, std::placeholders::_1));
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&LocalPlannerNode::odomCallback, this, std::placeholders::_1));
    this->costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("local_costmap", 1, std::bind(&LocalPlannerNode::costmapCallback, this, std::placeholders::_1));

    this->avoidance_enabled_ = this->declare_parameter<bool>("avoidance_enabled", true);
    this->odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    this->costmap_timeout_s_ = this->declare_parameter<double>("costmap_timeout_s", 0.5);
    this->stop_margin_m_ = this->declare_parameter<double>("stop_margin_m", 0.3);
    avoidance_params_.max_offset = this->declare_parameter<double>("max_lateral_offset", avoidance_params_.max_offset);
    avoidance_params_.ramp_length = this->declare_parameter<double>("avoidance_ramp_length", avoidance_params_.ramp_length);
    avoidance_params_.deviation_weight = this->declare_parameter<double>("deviation_weight", avoidance_params_.deviation_weight);
    avoidance_params_.change_weight = this->declare_parameter<double>("change_weight", avoidance_params_.change_weight);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

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

void LocalPlannerNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    costmap_.width = static_cast<int>(msg->info.width);
    costmap_.height = static_cast<int>(msg->info.height);
    costmap_.resolution = msg->info.resolution;
    costmap_.origin_x = msg->info.origin.position.x;
    costmap_.origin_y = msg->info.origin.position.y;
    costmap_.data = msg->data;
    costmap_time_ = this->now();
}

bool LocalPlannerNode::globalPathInOdomFrame(nav_msgs::msg::Path &path)
{
    if (path.header.frame_id.empty() || path.header.frame_id == odom_frame_)
    {
        return true;
    }

    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(odom_frame_, path.header.frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Cannot express the global path in %s: %s", odom_frame_.c_str(), e.what());
        return false;
    }

    const double yaw = tf2::getYaw(t.transform.rotation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (auto &pose : path.poses)
    {
        const double x = pose.pose.position.x, y = pose.pose.position.y;
        pose.pose.position.x = t.transform.translation.x + c * x - s * y;
        pose.pose.position.y = t.transform.translation.y + s * x + c * y;
        const double pose_yaw = tf2::getYaw(pose.pose.orientation) + yaw;
        pose.pose.orientation.z = std::sin(0.5 * pose_yaw);
        pose.pose.orientation.w = std::cos(0.5 * pose_yaw);
        pose.header.frame_id = odom_frame_;
    }
    path.header.frame_id = odom_frame_;
    return true;
}

void LocalPlannerNode::publishLocalPath()
{
    if (global_path_.poses.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Global path is empty. Cannot compute local path.");
        return;
    }

    nav_msgs::msg::Path transformed = global_path_;
    if (!globalPathInOdomFrame(transformed))
    {
        return;
    }
    const nav_msgs::msg::Path stored = global_path_;
    global_path_ = transformed;

    const bool costmap_fresh = costmap_.valid() && (this->now() - costmap_time_).seconds() < costmap_timeout_s_;
    obstacles_detected_ = avoidance_enabled_ && costmap_fresh;
    local_path_ = obstacles_detected_ ? computeLocalPath(lookahead_distance_, point_spacing_) : getLocalPath(lookahead_distance_, point_spacing_);
    global_path_ = stored;

    if (!local_path_.poses.empty())
    {
        local_path_pub_->publish(local_path_);
    }
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
    const nav_msgs::msg::Path base = getLocalPath(lookahead_distance, point_spacing);
    if (base.poses.size() < 2)
    {
        return base;
    }

    std::vector<ferrari_planning::PlannerPose> poses;
    poses.reserve(base.poses.size());
    for (const auto &p : base.poses)
    {
        poses.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
    }

    const double dx = odom_.pose.pose.position.x - poses.front().x;
    const double dy = odom_.pose.pose.position.y - poses.front().y;
    const double start_offset = -std::sin(poses.front().yaw) * dx + std::cos(poses.front().yaw) * dy;

    const auto result = ferrari_planning::planAvoidance(poses, costmap_, avoidance_params_, std::clamp(start_offset, -avoidance_params_.max_offset, avoidance_params_.max_offset), previous_offset_);
    if (!result.feasible)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Path blocked: stopping before the obstacle");
        const size_t collision = ferrari_planning::firstCollisionIndex(poses, costmap_, avoidance_params_);
        const size_t margin = static_cast<size_t>(std::ceil(stop_margin_m_ / std::max(point_spacing, 1e-3)));
        nav_msgs::msg::Path truncated = base;
        const size_t keep = collision > margin ? collision - margin : 0;
        if (keep < 2)
        {
            truncated.poses.clear();
        }
        else
        {
            truncated.poses.resize(std::min(keep, truncated.poses.size()));
        }
        return truncated;
    }
    previous_offset_ = result.offset;

    nav_msgs::msg::Path path = base;
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        path.poses[i].pose.position.x = result.path[i].x;
        path.poses[i].pose.position.y = result.path[i].y;
        path.poses[i].pose.orientation.z = std::sin(0.5 * result.path[i].yaw);
        path.poses[i].pose.orientation.w = std::cos(0.5 * result.path[i].yaw);
    }
    return path;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}