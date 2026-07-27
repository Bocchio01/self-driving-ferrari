#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LocalPlannerNode : public rclcpp::Node
{
public:
    explicit LocalPlannerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void localPathPublisher();

private:
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishLocalPath();

    void updateClosestIndex();
    const nav_msgs::msg::Path getLocalPath(double lookahead_distance, double point_spacing);
    const nav_msgs::msg::Path computeLocalPath(double lookahead_distance, double point_spacing);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path local_path_;
    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::Odometry odom_;

    std::size_t closest_path_index_;
    bool obstacles_detected_;
    double lookahead_distance_;
    double point_spacing_;
};
