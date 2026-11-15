#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "ferrari_mapping/slam_core.hpp"

using namespace ferrari_mapping;

namespace
{
    double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    Pose2D toPose2D(const geometry_msgs::msg::TransformStamped &t)
    {
        return {t.transform.translation.x, t.transform.translation.y, yawFromQuaternion(t.transform.rotation)};
    }
}

class SlamNode : public rclcpp::Node
{
public:
    SlamNode() : Node("slam_node")
    {
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "lidar_link");
        const std::string mode = this->declare_parameter<std::string>("mode", "mapping");
        const std::string map_file = this->declare_parameter<std::string>("map_file", "");
        save_path_ = this->declare_parameter<std::string>("map_save_path", "/tmp/ferrari_map");
        const double map_period = this->declare_parameter<double>("map_publish_period_s", 1.0);

        SlamParams params;
        params.grid.size_x_m = this->declare_parameter<double>("map_size_m", 50.0);
        params.grid.size_y_m = params.grid.size_x_m;
        params.grid.resolution = this->declare_parameter<double>("resolution", 0.05);
        params.grid.origin_x = -params.grid.size_x_m / 2.0;
        params.grid.origin_y = -params.grid.size_y_m / 2.0;
        params.matcher.linear_window = this->declare_parameter<double>("search_linear_window", params.matcher.linear_window);
        params.matcher.angular_window = this->declare_parameter<double>("search_angular_window", params.matcher.angular_window);
        params.matcher.min_score = this->declare_parameter<double>("min_match_score", params.matcher.min_score);
        params.keyframe_distance = this->declare_parameter<double>("keyframe_distance", params.keyframe_distance);
        params.mapping = mode != "localization";
        slam_ = std::make_unique<SlamCore>(params);

        if (!params.mapping)
        {
            OccupancyGrid grid(params.grid);
            if (map_file.empty() || !loadMap(map_file, grid))
            {
                RCLCPP_ERROR(get_logger(), "Localization mode needs a valid map_file, got '%s'", map_file.c_str());
            }
            else
            {
                slam_->setGrid(grid);
            }
        }

        Pose2D initial{this->declare_parameter<double>("initial_x", 0.0), this->declare_parameter<double>("initial_y", 0.0), this->declare_parameter<double>("initial_yaw", 0.0)};
        slam_->setMapToOdom(initial);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));
        save_srv_ = this->create_service<std_srvs::srv::Trigger>("save_map", std::bind(&SlamNode::saveCallback, this, std::placeholders::_1, std::placeholders::_2));
        tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SlamNode::publishTransform, this));
        map_timer_ = this->create_wall_timer(std::chrono::duration<double>(map_period), std::bind(&SlamNode::publishMap, this));
        stats_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&SlamNode::reportStatistics, this));

        RCLCPP_INFO(get_logger(), "SLAM started in %s mode", params.mapping ? "mapping" : "localization");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!lidar_ready_)
        {
            try
            {
                const auto t = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
                lidar_in_base_ = toPose2D(t);
                lidar_ready_ = true;
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for %s -> %s: %s", base_frame_.c_str(), lidar_frame_.c_str(), e.what());
                return;
            }
        }

        geometry_msgs::msg::TransformStamped odom_to_base;
        try
        {
            odom_to_base = tf_buffer_->lookupTransform(odom_frame_, base_frame_, msg->header.stamp, tf2::durationFromSec(0.05));
        }
        catch (const tf2::TransformException &)
        {
            try
            {
                odom_to_base = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
                ++stale_odometry_count_;
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No odometry transform: %s", e.what());
                return;
            }
        }

        std::vector<Eigen::Vector2d> points;
        points.reserve(msg->ranges.size());
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const double range = msg->ranges[i];
            if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max)
            {
                continue;
            }
            const double angle = msg->angle_min + msg->angle_increment * static_cast<double>(i);
            const Pose2D base_point = compose(lidar_in_base_, Pose2D{range * std::cos(angle), range * std::sin(angle), 0.0});
            points.emplace_back(base_point.x, base_point.y);
        }
        if (points.size() < 30)
        {
            return;
        }

        const SlamResult result = slam_->process(points, Eigen::Vector2d(lidar_in_base_.x, lidar_in_base_.y), toPose2D(odom_to_base));
        ++scan_count_;
        matched_count_ += result.matched ? 1 : 0;
        score_sum_ += result.score;
    }

    void publishTransform()
    {
        const Pose2D t = slam_->mapToOdom();
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        msg.child_frame_id = odom_frame_;
        msg.transform.translation.x = t.x;
        msg.transform.translation.y = t.y;
        msg.transform.rotation.z = std::sin(0.5 * t.yaw);
        msg.transform.rotation.w = std::cos(0.5 * t.yaw);
        tf_broadcaster_->sendTransform(msg);
    }

    void publishMap()
    {
        const OccupancyGrid &grid = slam_->grid();
        cv::Mat known = grid.logOdds() != 0.0f;
        if (cv::countNonZero(known) == 0)
        {
            return;
        }
        const cv::Rect box = cv::boundingRect(known);
        const auto data = grid.toOccupancyData();

        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        msg.info.resolution = static_cast<float>(grid.params().resolution);
        msg.info.width = box.width;
        msg.info.height = box.height;
        msg.info.origin.position.x = grid.params().origin_x + box.x * grid.params().resolution;
        msg.info.origin.position.y = grid.params().origin_y + box.y * grid.params().resolution;
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(static_cast<size_t>(box.width) * box.height);
        for (int y = 0; y < box.height; ++y)
        {
            for (int x = 0; x < box.width; ++x)
            {
                msg.data[static_cast<size_t>(y) * box.width + x] = data[static_cast<size_t>(box.y + y) * grid.width() + box.x + x];
            }
        }
        map_pub_->publish(msg);
    }

    void saveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = saveMap(slam_->grid(), save_path_);
        response->message = response->success ? "Map saved to " + save_path_ + ".yaml" : "Failed to save map to " + save_path_;
    }

    void reportStatistics()
    {
        if (scan_count_ == 0)
        {
            return;
        }
        RCLCPP_INFO(get_logger(), "slam: %lu scans, %.0f%% matched, mean score %.2f, %lu keyframes, %lu scans without exact odometry stamp", static_cast<unsigned long>(scan_count_),
                    100.0 * matched_count_ / scan_count_, score_sum_ / scan_count_, static_cast<unsigned long>(slam_->keyframeCount()), static_cast<unsigned long>(stale_odometry_count_));
    }

    std::string map_frame_, odom_frame_, base_frame_, lidar_frame_, save_path_;
    std::unique_ptr<SlamCore> slam_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
    rclcpp::TimerBase::SharedPtr tf_timer_, map_timer_, stats_timer_;

    Pose2D lidar_in_base_;
    bool lidar_ready_ = false;
    size_t scan_count_ = 0;
    size_t matched_count_ = 0;
    size_t stale_odometry_count_ = 0;
    double score_sum_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamNode>());
    rclcpp::shutdown();
    return 0;
}
