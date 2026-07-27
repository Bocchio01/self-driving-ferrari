#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "ferrari_vehicle/vehicle_model_factory.hpp"

using namespace vehicle_models;
using namespace vehicle_models::ackermann::kinematic_cartesian;
using std::placeholders::_1;

class FerrariTwinNode : public rclcpp::Node
{
public:
    FerrariTwinNode()
        : Node("ferrari_twin_node"),
          model_(Params{})
    {
        Params kc_params;
        kc_params.wheelbase = this->declare_parameter<double>("wheelbase", 2.5);
        kc_params.max_steer = this->declare_parameter<double>("max_steering_angle", 0.523); // Roughly 30 degrees in radians
        kc_params.max_speed = this->declare_parameter<double>("max_speed", 20.0);

        VehicleModelVariant model_variant = VehicleModelFactory::create(ModelType::ACKERMANN_KINEMATIC_CARTESIAN, kc_params);
        model_ = std::get<Model>(model_variant);

        cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10, std::bind(&FerrariTwinNode::cmd_callback, this, _1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt * 1000)),
            std::bind(&FerrariTwinNode::timer_callback, this));
    }

private:
    void cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        u_.speed = msg->drive.speed;
        u_.steering_angle = msg->drive.steering_angle;
    }

    void timer_callback()
    {
        auto x = Model::toVector(x_);
        auto u = Model::toVector(u_);

        x = model_.step(x, u, dt);
        x_ = Model::toStruct(x);

        // Publish Odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_.x;
        odom_msg.pose.pose.position.y = x_.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, x_.yaw);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = u_.speed;
        odom_msg.twist.twist.angular.z = (u_.speed / model_.params().wheelbase) * std::tan(u_.steering_angle);
        odom_pub_->publish(odom_msg);
    }

    Model model_;
    State x_;
    Command u_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    static constexpr double dt = 0.02;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FerrariTwinNode>());
    rclcpp::shutdown();
    return 0;
}