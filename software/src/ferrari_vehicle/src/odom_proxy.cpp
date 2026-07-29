#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "ferrari_vehicle/msg/odom_lite.hpp"

class OdomProxyNode : public rclcpp::Node
{
public:
    OdomProxyNode() : Node("odom_proxy_node")
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        odom_lite_sub_ = this->create_subscription<ferrari_vehicle::msg::OdomLite>("odom_lite", rclcpp::SensorDataQoS(), std::bind(&OdomProxyNode::odom_lite_callback, this, std::placeholders::_1));
    }

private:
    void odom_lite_callback(const ferrari_vehicle::msg::OdomLite::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom;
        odom.header = msg->header;
        odom.child_frame_id = msg->child_frame_id;

        odom.pose.pose = msg->pose;
        odom.twist.twist = msg->twist;

        odom_pub_->publish(odom);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<ferrari_vehicle::msg::OdomLite>::SharedPtr odom_lite_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomProxyNode>());
    rclcpp::shutdown();
    return 0;
}