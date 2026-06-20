#include "ferrari_control/control_gate.hpp"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

ControlGateNode::ControlGateNode(const rclcpp::NodeOptions &options)
    : Node("control_gate_node", options),
      gate_mode_(GateMode::TELEOP)
{
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 50.0);
    teleop_timeout_s_ = this->declare_parameter<double>("teleop_timeout_s", 0.25);
    auto_timeout_s_ = this->declare_parameter<double>("auto_timeout_s", 0.25);

    teleop_timeout_ = rclcpp::Duration::from_seconds(teleop_timeout_s_);
    auto_timeout_ = rclcpp::Duration::from_seconds(auto_timeout_s_);

    last_teleop_time_ = this->now();
    last_auto_time_ = this->now();
    last_teleop_cmd_ = makeStopCommand();
    last_auto_cmd_ = makeStopCommand();

    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);

    teleop_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/teleop_cmd", 10, std::bind(&ControlGateNode::teleopCmdCallback, this, std::placeholders::_1));
    auto_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/auto_cmd", 10, std::bind(&ControlGateNode::autoCmdCallback, this, std::placeholders::_1));

    switch_gate_mode_srv_ = this->create_service<std_srvs::srv::Trigger>("/switch_gate_mode", std::bind(&ControlGateNode::handleSwitchGateMode, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ControlGateNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "control_gate_node started in TELEOP mode");
}

void ControlGateNode::teleopCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    last_teleop_cmd_ = *msg;
    last_teleop_time_ = this->now();
}

void ControlGateNode::autoCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    last_auto_cmd_ = *msg;
    last_auto_time_ = this->now();
}

void ControlGateNode::controlLoop()
{
    ackermann_msgs::msg::AckermannDriveStamped output = makeStopCommand();

    if (gate_mode_ == GateMode::TELEOP)
    {
        if (isCommandFresh(last_teleop_time_, teleop_timeout_))
        {
            output = last_teleop_cmd_;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Teleop command timeout. Publishing stop command.");
        }
    }
    else // GateMode::AUTO
    {
        if (isCommandFresh(last_auto_time_, auto_timeout_))
        {
            output = last_auto_cmd_;
        }
    }

    output.header.stamp = this->now();
    output.header.frame_id = "base_link";
    cmd_pub_->publish(output);
}

void ControlGateNode::handleSwitchGateMode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused
    gate_mode_ = (gate_mode_ == GateMode::TELEOP) ? GateMode::AUTO : GateMode::TELEOP;

    response->success = true;
    response->message = (gate_mode_ == GateMode::AUTO) ? "Gate mode set to AUTO" : "Gate mode set to TELEOP";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

ackermann_msgs::msg::AckermannDriveStamped ControlGateNode::makeStopCommand() const
{
    ackermann_msgs::msg::AckermannDriveStamped cmd;

    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = 0.0;
    cmd.drive.speed = 0.0;

    return cmd;
}

bool ControlGateNode::isCommandFresh(const rclcpp::Time &stamp, const rclcpp::Duration &timeout) const
{
    return (this->now() - stamp) <= timeout;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlGateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
