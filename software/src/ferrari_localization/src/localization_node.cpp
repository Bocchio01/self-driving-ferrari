#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ferrari_localization/state_estimator.hpp"

using namespace ferrari_localization;

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {
        const std::string filter = this->declare_parameter<std::string>("filter", "ekf");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

        default_speed_variance_ = this->declare_parameter<double>("wheel_speed_variance", 0.0025);
        default_yaw_rate_variance_ = this->declare_parameter<double>("kinematic_yaw_rate_variance", 0.05);
        gyro_variance_ = this->declare_parameter<double>("gyro_variance", 0.25);
        standstill_gyro_variance_ = this->declare_parameter<double>("standstill_gyro_variance", 1.0e-2);
        standstill_speed_ = this->declare_parameter<double>("standstill_speed", 0.02);
        max_predict_dt_ = this->declare_parameter<double>("max_predict_dt", 0.2);
        wheel_timeout_ = this->declare_parameter<double>("wheel_timeout", 0.3);
        const double rate_hz = this->declare_parameter<double>("rate_hz", 50.0);

        NoiseDensities noise;
        noise.position = this->declare_parameter<double>("noise_position", 0.01);
        noise.speed = this->declare_parameter<double>("noise_speed", 1.0);
        noise.yaw_rate = this->declare_parameter<double>("noise_yaw_rate", 1.0);
        noise.gyro_bias = this->declare_parameter<double>("noise_gyro_bias", 0.001);

        InitialUncertainty initial;
        initial.pose = this->declare_parameter<double>("initial_pose_std", 0.01);
        initial.speed = this->declare_parameter<double>("initial_speed_std", 0.3);
        initial.yaw_rate = this->declare_parameter<double>("initial_yaw_rate_std", 0.5);
        initial.gyro_bias = this->declare_parameter<double>("initial_gyro_bias_std", 0.1);

        if (filter == "pf")
        {
            const int particles = this->declare_parameter<int>("num_particles", 500);
            const int seed = this->declare_parameter<int>("seed", 0);
            estimator_ = std::make_unique<ParticleEstimator>(static_cast<size_t>(particles), noise, initial, static_cast<unsigned int>(seed));
        }
        else
        {
            estimator_ = std::make_unique<EkfEstimator>(noise, initial);
        }
        RCLCPP_INFO(this->get_logger(), "Localization filter: %s", filter == "pf" ? "particle filter" : "extended kalman filter");

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        if (publish_tf_)
        {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 20, std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));
        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("wheel_odom", 20, std::bind(&LocalizationNode::wheelOdomCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), std::bind(&LocalizationNode::step, this));
    }

private:
    void step()
    {
        if (!started_)
        {
            return;
        }

        const rclcpp::Time now = this->now();
        const double dt = std::min((now - last_step_time_).seconds(), max_predict_dt_);
        last_step_time_ = now;
        if (dt <= 0.0)
        {
            return;
        }

        estimator_->predict(dt);
        publish(now);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!started_)
        {
            return;
        }

        const double gyro_z = msg->angular_velocity.z;
        const bool wheel_fresh = (this->now() - last_wheel_time_).seconds() < wheel_timeout_;
        if (wheel_fresh && std::abs(last_wheel_speed_) < standstill_speed_)
        {
            estimator_->updateGyroBias(gyro_z, standstill_gyro_variance_);
        }
        else
        {
            estimator_->updateGyro(gyro_z, gyro_variance_);
        }
    }

    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_wheel_speed_ = msg->twist.twist.linear.x;
        last_wheel_time_ = this->now();

        if (!started_)
        {
            started_ = true;
            last_step_time_ = last_wheel_time_;
            return;
        }

        const double speed_variance = msg->twist.covariance[0] > 0.0 ? msg->twist.covariance[0] : default_speed_variance_;
        const double yaw_rate_variance = msg->twist.covariance[35] > 0.0 ? msg->twist.covariance[35] : default_yaw_rate_variance_;

        estimator_->updateWheelSpeed(last_wheel_speed_, speed_variance);
        estimator_->updateKinematicYawRate(msg->twist.twist.angular.z, yaw_rate_variance);
    }

    void publish(const rclcpp::Time &stamp)
    {
        const Estimate e = estimator_->estimate();
        const double yaw = e.state(YAW);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = e.state(X);
        odom.pose.pose.position.y = e.state(Y);
        odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
        odom.pose.pose.orientation.w = std::cos(0.5 * yaw);
        odom.twist.twist.linear.x = e.state(SPEED);
        odom.twist.twist.angular.z = e.state(YAW_RATE);

        odom.pose.covariance.fill(0.0);
        odom.twist.covariance.fill(0.0);
        for (size_t i : {14, 21, 28})
        {
            odom.pose.covariance[i] = 1.0e6;
        }
        for (size_t i : {7, 14, 21, 28})
        {
            odom.twist.covariance[i] = 1.0e6;
        }
        odom.pose.covariance[0] = e.covariance(X, X);
        odom.pose.covariance[1] = e.covariance(X, Y);
        odom.pose.covariance[6] = e.covariance(Y, X);
        odom.pose.covariance[7] = e.covariance(Y, Y);
        odom.pose.covariance[5] = e.covariance(X, YAW);
        odom.pose.covariance[30] = e.covariance(YAW, X);
        odom.pose.covariance[11] = e.covariance(Y, YAW);
        odom.pose.covariance[31] = e.covariance(YAW, Y);
        odom.pose.covariance[35] = e.covariance(YAW, YAW);
        odom.twist.covariance[0] = e.covariance(SPEED, SPEED);
        odom.twist.covariance[35] = e.covariance(YAW_RATE, YAW_RATE);
        odom_pub_->publish(odom);

        if (tf_broadcaster_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = e.state(X);
            tf.transform.translation.y = e.state(Y);
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    std::unique_ptr<StateEstimator> estimator_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;

    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_ = true;

    double default_speed_variance_ = 0.0;
    double default_yaw_rate_variance_ = 0.0;
    double gyro_variance_ = 0.0;
    double standstill_gyro_variance_ = 0.0;
    double standstill_speed_ = 0.0;
    double max_predict_dt_ = 0.0;
    double wheel_timeout_ = 0.0;

    bool started_ = false;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_step_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_wheel_time_{0, 0, RCL_ROS_TIME};
    double last_wheel_speed_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
