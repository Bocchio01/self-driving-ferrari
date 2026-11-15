#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>

#include <tf2/LinearMath/Quaternion.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "ferrari_vehicle/vehicle_model_factory.hpp"
#include "ferrari_vehicle/msg/odom_lite.hpp"

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

        speed_scale_error_ = this->declare_parameter<double>("odom_speed_scale_error", 0.0);
        steering_offset_ = this->declare_parameter<double>("odom_steering_offset", 0.0);
        speed_noise_std_ = this->declare_parameter<double>("odom_speed_noise_std", 0.0);
        yaw_rate_noise_std_ = this->declare_parameter<double>("odom_yaw_rate_noise_std", 0.0);

        VehicleModelVariant model_variant = VehicleModelFactory::create(ModelType::ACKERMANN_KINEMATIC_CARTESIAN, kc_params);
        model_ = std::get<Model>(model_variant);

        cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10, std::bind(&FerrariTwinNode::cmd_callback, this, _1));
        odom_pub_ = this->create_publisher<ferrari_vehicle::msg::OdomLite>("odom_lite", rclcpp::SensorDataQoS());
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", rclcpp::SensorDataQoS());
        truth_pub_ = this->create_publisher<ferrari_vehicle::msg::OdomLite>("ground_truth", 10);

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

        const double true_yaw_rate = (u_.speed / model_.params().wheelbase) * std::tan(u_.steering_angle);
        const bool moving = std::abs(u_.speed) > 1e-3;
        const double measured_speed = u_.speed * (1.0 + speed_scale_error_) + (moving ? speed_noise_std_ * normal_(rng_) : 0.0);
        const double measured_yaw_rate = (u_.speed / model_.params().wheelbase) * std::tan(u_.steering_angle + steering_offset_) + (moving ? yaw_rate_noise_std_ * normal_(rng_) : 0.0);
        const bool drifting = speed_scale_error_ != 0.0 || steering_offset_ != 0.0 || speed_noise_std_ != 0.0 || yaw_rate_noise_std_ != 0.0;
        if (drifting)
        {
            const double yaw_mid = odom_yaw_ + 0.5 * measured_yaw_rate * dt;
            odom_x_ += measured_speed * std::cos(yaw_mid) * dt;
            odom_y_ += measured_speed * std::sin(yaw_mid) * dt;
            odom_yaw_ += measured_yaw_rate * dt;
        }
        else
        {
            odom_x_ = x_.x;
            odom_y_ = x_.y;
            odom_yaw_ = x_.yaw;
        }

        const auto stamp = this->get_clock()->now();
        auto make_message = [&stamp](double px, double py, double yaw, double v, double w)
        {
            auto msg = ferrari_vehicle::msg::OdomLite();
            msg.header.stamp = stamp;
            msg.header.frame_id = "odom";
            msg.child_frame_id = "base_link";
            msg.pose.position.x = px;
            msg.pose.position.y = py;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();
            msg.twist.linear.x = v;
            msg.twist.angular.z = w;
            return msg;
        };

        auto odom_msg = make_message(odom_x_, odom_y_, odom_yaw_, measured_speed, measured_yaw_rate);
        odom_pub_->publish(odom_msg);
        truth_pub_->publish(make_message(x_.x, x_.y, x_.yaw, u_.speed, true_yaw_rate));

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = odom_msg.header.stamp;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation_covariance[0] = -1.0;
        imu_msg.angular_velocity.z = true_yaw_rate;
        imu_msg.linear_acceleration.z = 9.80665;
        for (size_t i = 0; i < 3; ++i)
        {
            imu_msg.angular_velocity_covariance[i * 4] = 1.0e-4;
            imu_msg.linear_acceleration_covariance[i * 4] = 2.5e-3;
        }
        imu_pub_->publish(imu_msg);
    }

    Model model_;
    State x_;
    Command u_;

    double speed_scale_error_ = 0.0;
    double steering_offset_ = 0.0;
    double speed_noise_std_ = 0.0;
    double yaw_rate_noise_std_ = 0.0;
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_yaw_ = 0.0;
    std::mt19937 rng_{42};
    std::normal_distribution<double> normal_{0.0, 1.0};
    rclcpp::Publisher<ferrari_vehicle::msg::OdomLite>::SharedPtr truth_pub_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
    rclcpp::Publisher<ferrari_vehicle::msg::OdomLite>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
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