#include <array>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ferrari_vehicle/msg/odom_lite.hpp"

class OdomProxyNode : public rclcpp::Node
{
public:
    OdomProxyNode() : Node("odom_proxy_node")
    {
        const double vx_variance = this->declare_parameter<double>("odom_vx_variance", 0.0025);
        const double vyaw_variance = this->declare_parameter<double>("odom_vyaw_variance", 0.05);
        const double pose_variance = this->declare_parameter<double>("odom_pose_variance", 0.1);

        for (size_t i : {0, 7, 14, 21, 28, 35})
        {
            pose_covariance_[i] = pose_variance;
            twist_covariance_[i] = 1.0e3;
        }
        twist_covariance_[0] = vx_variance;
        twist_covariance_[35] = vyaw_variance;

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        odom_lite_sub_ = this->create_subscription<ferrari_vehicle::msg::OdomLite>("odom_lite", rclcpp::SensorDataQoS(), std::bind(&OdomProxyNode::odom_lite_callback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_raw", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg)
                                                                    { imu_pub_->publish(*msg); });
    }

private:
    void odom_lite_callback(const ferrari_vehicle::msg::OdomLite::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom;
        odom.header = msg->header;
        odom.child_frame_id = msg->child_frame_id;

        odom.pose.pose = msg->pose;
        odom.twist.twist = msg->twist;
        odom.pose.covariance = pose_covariance_;
        odom.twist.covariance = twist_covariance_;

        odom_pub_->publish(odom);
    }

    std::array<double, 36> pose_covariance_{};
    std::array<double, 36> twist_covariance_{};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<ferrari_vehicle::msg::OdomLite>::SharedPtr odom_lite_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomProxyNode>());
    rclcpp::shutdown();
    return 0;
}
