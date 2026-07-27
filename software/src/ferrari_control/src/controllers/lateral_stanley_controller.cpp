#include <cmath>
#include <algorithm>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ferrari_control/trajectory_utils.hpp"
#include "ferrari_control/controllers/lateral_stanley_controller.hpp"

LateralStanleyControllerNode::LateralStanleyControllerNode(const rclcpp::NodeOptions &options)
    : Controller("lateral_stanley_cmd", "lateral_stanley_controller_node", options)
{
    this->declare_parameter<double>("k_gain", 1.5); // Cross-track error gain
    this->declare_parameter<double>("k_soft", 1.0); // Softening constant to prevent division by zero
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LateralStanleyControllerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    auto ret = Controller::on_configure(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    k_gain_ = this->get_parameter("k_gain").as_double();
    k_soft_ = this->get_parameter("k_soft").as_double();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LateralStanleyControllerNode::computeControlCommand()
{
    std::lock_guard<std::mutex> lock(local_trajectory_mutex_);
    if (local_trajectory_.points.empty())
        return;

    double current_velocity = odom_.twist.twist.linear.x;
    double yaw = tf2::getYaw(odom_.pose.pose.orientation);

    // 1. Project the rear-axle pose to the front-axle pose
    geometry_msgs::msg::Pose front_axle_pose = odom_.pose.pose;
    front_axle_pose.position.x += wheelbase_ * std::cos(yaw);
    front_axle_pose.position.y += wheelbase_ * std::sin(yaw);

    // 2. Compute the spatial horizon and Frenet errors relative to the FRONT axle
    auto horizon = ferrari_control::trajectory_utils::getTrajectoryHorizonBySpace(front_axle_pose, local_trajectory_, 2);
    auto frenet_err = ferrari_control::trajectory_utils::computeFrenetError(front_axle_pose, horizon);

    // 3. Stanley control law
    double cross_track_steering = std::atan2(k_gain_ * frenet_err.d, current_velocity + k_soft_);
    double steering_angle = -frenet_err.mu - cross_track_steering;

    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steering_angle;

    publishCmd(cmd);
}

rcl_interfaces::msg::SetParametersResult LateralStanleyControllerNode::onParamsChanged(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "k_gain")
        {
            k_gain_ = param.as_double();
        }
        else if (param.get_name() == "k_soft")
        {
            k_soft_ = param.as_double();
        }
    }
    return result;
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<LateralStanleyControllerNode> node = std::make_shared<LateralStanleyControllerNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}