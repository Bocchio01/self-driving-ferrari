#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class ControllerMultiplexerNode : public rclcpp::Node
{
public:
    explicit ControllerMultiplexerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    struct CommandState
    {
        double speed{0.0};
        double steering_angle{0.0};
        rclcpp::Time speed_timestamp{0, 0, RCL_ROS_TIME};
        rclcpp::Time steering_timestamp{0, 0, RCL_ROS_TIME};
    };

    struct ControllerConfig
    {
        std::string node_name;
        std::string topic_name;
        bool provides_lateral;
        bool provides_longitudinal;
    };

    struct ModeConfig
    {
        std::string name;
        std::vector<std::string> controller_names; // keys into controllers_
    };

    // What we currently believe about a single physical controller node.
    // UNAVAILABLE: its lifecycle service isn't discoverable right now.
    // UNCONFIGURED/INACTIVE/ACTIVE: mirror lifecycle_msgs primary states we care about.
    enum class LifecycleState
    {
        UNAVAILABLE,
        UNCONFIGURED,
        INACTIVE,
        ACTIVE
    };

    struct ControllerRuntime
    {
        ControllerConfig cfg;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
        LifecycleState state{LifecycleState::UNAVAILABLE};
        bool transition_in_flight{false};
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub;
    };

    void initializeModes();
    void controlLoop();
    void reconcileControllers();
    void requestTransition(const std::string &node_name, uint8_t transition_id);
    bool isDesired(const std::string &node_name) const;
    void subscribeController(ControllerRuntime &rt);

    void handleSwitchController(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    std::vector<ModeConfig> modes_;
    size_t current_mode_idx_{0};

    // One entry per unique physical controller node, shared across modes that reference it.
    std::unordered_map<std::string, ControllerRuntime> controllers_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_controller_srv_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr reconcile_timer_;

    CommandState command_state_;
    rclcpp::Duration control_timeout_{0, 0};
};