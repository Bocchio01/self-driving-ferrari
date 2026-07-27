#include <cmath>
#include <algorithm>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "ferrari_control/trajectory_utils.hpp"
#include "ferrari_control/controllers/lateral_pure_pursuit_controller.hpp"

LateralPurePursuitControllerNode::LateralPurePursuitControllerNode(const rclcpp::NodeOptions &options)
    : Controller("lateral_pure_pursuit_cmd", "lateral_pure_pursuit_controller_node", options)
{
    this->declare_parameter<double>("min_lookahead", 2.0);
    this->declare_parameter<double>("lookahead_gain", 0.1);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LateralPurePursuitControllerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    auto ret = Controller::on_configure(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    min_lookahead_ = this->get_parameter("min_lookahead").as_double();
    lookahead_gain_ = this->get_parameter("lookahead_gain").as_double();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LateralPurePursuitControllerNode::computeControlCommand()
{
    std::lock_guard<std::mutex> lock(local_trajectory_mutex_);
    if (local_trajectory_.points.empty())
        return;

    // Calculate dynamic lookahead based on the live speed
    double lookahead_distance = std::max(min_lookahead_, lookahead_gain_ * odom_.twist.twist.linear.x);

    auto target_point = ferrari_control::trajectory_utils::getLookaheadPoint(
        local_trajectory_,
        odom_.pose.pose,
        lookahead_distance);

    // Pure pursuit steering calculation using LOCAL coordinates
    double alpha = std::atan2(target_point.y, target_point.x);
    double steering_angle = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance);

    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steering_angle;

    publishCmd(cmd);
}

// Override the virtual parameter callback from the base Controller class
rcl_interfaces::msg::SetParametersResult LateralPurePursuitControllerNode::onParamsChanged(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "min_lookahead")
        {
            min_lookahead_ = param.as_double();
        }
        else if (param.get_name() == "lookahead_gain")
        {
            lookahead_gain_ = param.as_double();
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<LateralPurePursuitControllerNode> node = std::make_shared<LateralPurePursuitControllerNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}