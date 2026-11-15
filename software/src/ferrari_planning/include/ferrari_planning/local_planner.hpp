#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "ferrari_planning/avoidance.hpp"

class LocalPlannerNode : public rclcpp::Node
{
public:
    explicit LocalPlannerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void localPathPublisher();

private:
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool globalPathInOdomFrame(nav_msgs::msg::Path &path);
    void publishLocalPath();

    void updateClosestIndex();
    const nav_msgs::msg::Path getLocalPath(double lookahead_distance, double point_spacing);
    const nav_msgs::msg::Path computeLocalPath(double lookahead_distance, double point_spacing);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path local_path_;
    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::Odometry odom_;

    std::size_t closest_path_index_;
    bool obstacles_detected_;
    bool avoidance_enabled_ = true;
    std::string odom_frame_ = "odom";
    double costmap_timeout_s_ = 0.5;
    double stop_margin_m_ = 0.3;
    double previous_offset_ = 0.0;
    ferrari_planning::CostGrid costmap_;
    rclcpp::Time costmap_time_{0, 0, RCL_ROS_TIME};
    ferrari_planning::AvoidanceParams avoidance_params_;
    double lookahead_distance_;
    double point_spacing_;
};
