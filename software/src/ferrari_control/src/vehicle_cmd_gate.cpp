#include "ferrari_control/vehicle_cmd_gate.hpp"

#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

VehicleCmdGateNode::VehicleCmdGateNode(const rclcpp::NodeOptions &options)
    : Node("vehicle_cmd_gate_node", options),
      gate_mode_(GateMode::TELEOP),
      armed_(false)
{
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 50.0);
    teleop_timeout_s_ = this->declare_parameter<double>("teleop_timeout_s", 0.25);
    auto_timeout_s_ = this->declare_parameter<double>("auto_timeout_s", 0.25);
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "base_link");

    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd_ackermann", 10);

    teleop_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/teleop_cmd", 10, std::bind(&VehicleCmdGateNode::teleopCmdCallback, this, _1));
    auto_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/auto_cmd", 10, std::bind(&VehicleCmdGateNode::autoCmdCallback, this, _1));

    toggle_gate_mode_srv_ = this->create_service<std_srvs::srv::Trigger>("/toggle_gate_mode", std::bind(&VehicleCmdGateNode::handleToggleGateMode, this, _1, _2));
    toggle_arm_srv_ = this->create_service<std_srvs::srv::Trigger>("/toggle_arm", std::bind(&VehicleCmdGateNode::handleToggleArm, this, _1, _2));

    last_teleop_time_ = this->now();
    last_auto_time_ = this->now();
    last_teleop_cmd_ = makeStopCommand();
    last_auto_cmd_ = makeStopCommand();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&VehicleCmdGateNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "vehicle_cmd_gate_node started in TELEOP mode (disarmed)");
}

void VehicleCmdGateNode::teleopCmdCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    last_teleop_cmd_ = *msg;
    last_teleop_time_ = this->now();
}

void VehicleCmdGateNode::autoCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    last_auto_cmd_ = *msg;
    last_auto_time_ = this->now();
}

void VehicleCmdGateNode::controlLoop()
{
    const auto now = this->now();
    const rclcpp::Duration teleop_timeout = rclcpp::Duration::from_seconds(teleop_timeout_s_);
    const rclcpp::Duration auto_timeout = rclcpp::Duration::from_seconds(auto_timeout_s_);

    ackermann_msgs::msg::AckermannDriveStamped output = makeStopCommand();

    if (!armed_)
    {
        cmd_pub_->publish(output);
        return;
    }

    if (gate_mode_ == GateMode::TELEOP)
    {
        if (isCommandFresh(last_teleop_time_, teleop_timeout))
        {
            output = last_teleop_cmd_;
        }
    }
    else
    {
        if (isCommandFresh(last_auto_time_, auto_timeout))
        {
            output = last_auto_cmd_;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "AUTO command timeout: forcing stop for safety");
        }
    }

    output.header.stamp = now;
    if (output.header.frame_id.empty())
    {
        output.header.frame_id = output_frame_id_;
    }
    cmd_pub_->publish(output);
}

void VehicleCmdGateNode::handleToggleGateMode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused
    gate_mode_ = (gate_mode_ == GateMode::TELEOP) ? GateMode::AUTO : GateMode::TELEOP;

    response->success = true;
    response->message = (gate_mode_ == GateMode::AUTO) ? "Gate mode set to AUTO" : "Gate mode set to TELEOP";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void VehicleCmdGateNode::handleToggleArm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused
    armed_ = !armed_;

    response->success = true;
    response->message = armed_ ? "Vehicle ARMED" : "Vehicle DISARMED";

    if (!armed_)
    {
        cmd_pub_->publish(makeStopCommand());
    }

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

ackermann_msgs::msg::AckermannDriveStamped VehicleCmdGateNode::makeStopCommand() const
{
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = output_frame_id_;
    cmd.drive.steering_angle = 0.0;
    cmd.drive.speed = 0.0;
    return cmd;
}

bool VehicleCmdGateNode::isCommandFresh(const rclcpp::Time &stamp, const rclcpp::Duration &timeout) const
{
    return (this->now() - stamp) <= timeout;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleCmdGateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
