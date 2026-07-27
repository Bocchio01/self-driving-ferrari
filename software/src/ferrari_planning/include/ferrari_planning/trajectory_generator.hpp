#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ferrari_planning/msg/trajectory.hpp>

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
    explicit TrajectoryGeneratorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Publisher<ferrari_planning::msg::Trajectory>::SharedPtr trajectory_pub_;

    double max_steering_angle_;
    double max_speed_;
    double max_lateral_acceleration_;
    double max_longitudinal_acceleration_;
    double max_longitudinal_deceleration_;
};
