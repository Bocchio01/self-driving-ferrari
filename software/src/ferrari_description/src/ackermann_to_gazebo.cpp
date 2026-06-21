#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class AckermannToGazeboNode : public rclcpp::Node
{
public:
    AckermannToGazeboNode() : Node("ackermann_to_gazebo")
    {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd", 10,
            [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
            {
                geometry_msgs::msg::Twist twist_msg;

                twist_msg.linear.x = msg->drive.speed;
                twist_msg.angular.z = msg->drive.steering_angle;

                twist_pub_->publish(twist_msg);
            });

        RCLCPP_INFO(this->get_logger(), "Manual Simulation Bridge: Ackermann -> Twist started");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToGazeboNode>());
    rclcpp::shutdown();
    return 0;
}