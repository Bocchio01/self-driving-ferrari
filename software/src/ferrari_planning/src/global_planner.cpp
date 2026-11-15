#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "ferrari_planning/hybrid_astar.hpp"

using namespace ferrari_planning;

namespace
{
    double yawOf(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    CostGrid toCostGrid(const nav_msgs::msg::OccupancyGrid &msg)
    {
        CostGrid grid;
        grid.width = static_cast<int>(msg.info.width);
        grid.height = static_cast<int>(msg.info.height);
        grid.resolution = msg.info.resolution;
        grid.origin_x = msg.info.origin.position.x;
        grid.origin_y = msg.info.origin.position.y;
        grid.data = msg.data;
        return grid;
    }

    CostGrid crop(const CostGrid &grid, double min_x, double min_y, double max_x, double max_y)
    {
        int x0 = static_cast<int>(std::floor((min_x - grid.origin_x) / grid.resolution));
        int y0 = static_cast<int>(std::floor((min_y - grid.origin_y) / grid.resolution));
        int x1 = static_cast<int>(std::ceil((max_x - grid.origin_x) / grid.resolution));
        int y1 = static_cast<int>(std::ceil((max_y - grid.origin_y) / grid.resolution));
        x0 = std::clamp(x0, 0, grid.width);
        x1 = std::clamp(x1, 0, grid.width);
        y0 = std::clamp(y0, 0, grid.height);
        y1 = std::clamp(y1, 0, grid.height);

        CostGrid out;
        out.resolution = grid.resolution;
        out.width = x1 - x0;
        out.height = y1 - y0;
        out.origin_x = grid.origin_x + x0 * grid.resolution;
        out.origin_y = grid.origin_y + y0 * grid.resolution;
        out.data.resize(static_cast<size_t>(std::max(out.width, 0)) * std::max(out.height, 0));
        for (int y = 0; y < out.height; ++y)
        {
            std::copy_n(grid.data.begin() + static_cast<size_t>(y0 + y) * grid.width + x0, out.width, out.data.begin() + static_cast<size_t>(y) * out.width);
        }
        return out;
    }
}

class GlobalPlannerNode : public rclcpp::Node
{
public:
    GlobalPlannerNode() : Node("global_planner_node")
    {
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        crop_margin_ = this->declare_parameter<double>("crop_margin", 5.0);
        max_failures_ = this->declare_parameter<int>("max_failures", 3);

        params_.wheelbase = this->declare_parameter<double>("wheelbase", params_.wheelbase);
        params_.max_steering_angle = this->declare_parameter<double>("max_steering_angle", params_.max_steering_angle);
        params_.step = this->declare_parameter<double>("step", params_.step);
        params_.goal_tolerance = this->declare_parameter<double>("goal_tolerance", params_.goal_tolerance);
        params_.use_goal_yaw = this->declare_parameter<bool>("use_goal_yaw", params_.use_goal_yaw);
        params_.cost_weight = this->declare_parameter<double>("cost_weight", params_.cost_weight);
        params_.max_expansions = this->declare_parameter<int>("max_expansions", params_.max_expansions);
        const double period = this->declare_parameter<double>("replan_period_s", 1.0);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", rclcpp::QoS(1).transient_local());
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap", rclcpp::QoS(1).transient_local(), [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
                                                                              { costmap_ = toCostGrid(*msg); });
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::duration<double>(period), std::bind(&GlobalPlannerNode::plan, this));
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped in_map = *msg;
        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_)
        {
            try
            {
                in_map = tf_buffer_->transform(*msg, map_frame_, tf2::durationFromSec(0.2));
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN(get_logger(), "Cannot express the goal in %s: %s", map_frame_.c_str(), e.what());
                return;
            }
        }
        goal_ = PlannerPose{in_map.pose.position.x, in_map.pose.position.y, yawOf(in_map.pose.orientation)};
        failures_ = 0;
        RCLCPP_INFO(get_logger(), "New goal (%.2f, %.2f) in %s", goal_->x, goal_->y, map_frame_.c_str());
        plan();
    }

    void publishPath(const std::vector<PlannerPose> &poses)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = map_frame_;
        for (const auto &p : poses)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.orientation.z = std::sin(0.5 * p.yaw);
            pose.pose.orientation.w = std::cos(0.5 * p.yaw);
            path.poses.push_back(pose);
        }
        path_pub_->publish(path);
    }

    void plan()
    {
        if (!goal_ || !costmap_.valid())
        {
            return;
        }

        PlannerPose start;
        try
        {
            const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            start = {t.transform.translation.x, t.transform.translation.y, yawOf(t.transform.rotation)};
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No pose in %s: %s", map_frame_.c_str(), e.what());
            return;
        }

        if (std::hypot(start.x - goal_->x, start.y - goal_->y) <= params_.goal_tolerance)
        {
            RCLCPP_INFO(get_logger(), "Goal reached");
            goal_.reset();
            publishPath({});
            return;
        }

        const double margin = crop_margin_;
        const CostGrid local = crop(costmap_, std::min(start.x, goal_->x) - margin, std::min(start.y, goal_->y) - margin, std::max(start.x, goal_->x) + margin, std::max(start.y, goal_->y) + margin);

        const auto t0 = std::chrono::steady_clock::now();
        const PlanResult result = planHybridAStar(local, start, *goal_, params_);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        if (result.success)
        {
            failures_ = 0;
            publishPath(result.path);
            RCLCPP_DEBUG(get_logger(), "Planned %zu poses in %.0f ms (%d expansions)", result.path.size(), ms, result.expansions);
        }
        else
        {
            ++failures_;
            RCLCPP_WARN(get_logger(), "No path to the goal (%d expansions, %.0f ms)", result.expansions, ms);
            if (failures_ >= max_failures_)
            {
                publishPath({});
            }
        }
    }

    std::string map_frame_, base_frame_;
    double crop_margin_ = 5.0;
    int max_failures_ = 3;
    int failures_ = 0;
    HybridAStarParams params_;
    CostGrid costmap_;
    std::optional<PlannerPose> goal_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
