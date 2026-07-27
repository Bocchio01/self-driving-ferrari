#include <algorithm>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "ferrari_control/trajectory_utils.hpp"
#include "ferrari_control/controllers/longitudinal_pid_controller.hpp"

LongitudinalPidControllerNode::LongitudinalPidControllerNode(const rclcpp::NodeOptions &options)
    : Controller("longitudinal_pid_cmd", "longitudinal_pid_controller_node", options),
      last_time_(0, 0, RCL_ROS_TIME)
{
    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<double>("ki", 0.1);
    this->declare_parameter<double>("kd", 0.05);
    this->declare_parameter<double>("max_acceleration", 3.0);
    this->declare_parameter<double>("max_braking", -5.0);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LongitudinalPidControllerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    auto ret = Controller::on_configure(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    max_acceleration_ = this->get_parameter("max_acceleration").as_double();
    max_braking_ = this->get_parameter("max_braking").as_double();

    // Reset PID states upon configuration
    integral_error_ = 0.0;
    previous_error_ = 0.0;
    last_time_ = this->now();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LongitudinalPidControllerNode::computeControlCommand()
{
    std::lock_guard<std::mutex> lock(local_trajectory_mutex_);
    if (local_trajectory_.points.empty())
        return;

    // Extract current state
    double current_velocity = odom_.twist.twist.linear.x;

    // Use spatial horizon to find the immediate reference point
    auto horizon = ferrari_control::trajectory_utils::getTrajectoryHorizonBySpace(odom_.pose.pose, local_trajectory_, 2);
    double target_velocity = horizon.points.empty() ? 0.0 : horizon.points.front().velocity.linear.x;

    double error = target_velocity - current_velocity;

    // PID terms
    integral_error_ += error * control_period_.count();
    double derivative = (error - previous_error_) / control_period_.count();
    previous_error_ = error;

    double acceleration_cmd = (kp_ * error) + (ki_ * integral_error_) + (kd_ * derivative);

    // Apply acceleration to compute the final speed command
    double speed_cmd = current_velocity + (acceleration_cmd * control_period_.count());
    speed_cmd = std::clamp(speed_cmd, -max_speed_, max_speed_);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.drive.speed = speed_cmd;

    // Steering is ignored by the longitudinal controller

    publishCmd(cmd);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<LongitudinalPidControllerNode> node = std::make_shared<LongitudinalPidControllerNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}