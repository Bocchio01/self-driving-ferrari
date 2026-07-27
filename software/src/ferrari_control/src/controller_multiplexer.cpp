#include "ferrari_control/controller_multiplexer.hpp"
#include <lifecycle_msgs/msg/transition.hpp>

using namespace std::chrono_literals;

ControllerMultiplexerNode::ControllerMultiplexerNode(const rclcpp::NodeOptions &options)
    : Node("control_multiplexer_node", options)
{
    double control_rate_hz = this->declare_parameter<double>("control_rate_hz", 50.0);
    double control_timeout_s = this->declare_parameter<double>("control_timeout_s", 0.25);
    control_timeout_ = rclcpp::Duration::from_seconds(control_timeout_s);

    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);
    switch_controller_srv_ = this->create_service<std_srvs::srv::Trigger>("switch_controller", std::bind(&ControllerMultiplexerNode::handleSwitchController, this, std::placeholders::_1, std::placeholders::_2));

    initializeModes();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz));
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ControllerMultiplexerNode::controlLoop, this));

    // Reconciliation runs independently of the control loop rate — 2 Hz is plenty
    // for lifecycle bookkeeping and keeps it cheap.
    reconcile_timer_ = this->create_wall_timer(500ms, std::bind(&ControllerMultiplexerNode::reconcileControllers, this));
    RCLCPP_INFO(this->get_logger(), "control_multiplexer_node started, target mode: %s", modes_[current_mode_idx_].name.c_str());
}

void ControllerMultiplexerNode::initializeModes()
{
    auto add_controller = [this](const std::string &node_name, const std::string &topic,
                                 bool lateral, bool longitudinal)
    {
        if (controllers_.find(node_name) == controllers_.end())
        {
            ControllerRuntime rt;
            rt.cfg = {node_name, topic, lateral, longitudinal};
            rt.client = this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
            controllers_.emplace(node_name, std::move(rt));
        }
    };

    add_controller("joypad_controller_node", "joypad_cmd", true, true);
    add_controller("mpc_controller_node", "mpc_cmd", true, true);
    add_controller("lateral_mpc_controller_node", "lateral_mpc_cmd", true, false);
    add_controller("lateral_stanley_controller_node", "lateral_stanley_cmd", true, false);
    add_controller("lateral_pure_pursuit_controller_node", "lateral_pure_pursuit_cmd", true, false);
    add_controller("longitudinal_pid_controller_node", "longitudinal_pid_cmd", false, true);

    modes_ = {
        {"JOYPAD", {"joypad_controller_node"}},
        {"MPC", {"mpc_controller_node"}},
        {"PID_MPC", {"lateral_mpc_controller_node", "longitudinal_pid_controller_node"}},
        {"PID_STANLEY", {"lateral_stanley_controller_node", "longitudinal_pid_controller_node"}},
        {"PID_PURE_PURSUIT", {"lateral_pure_pursuit_controller_node", "longitudinal_pid_controller_node"}}};
}

bool ControllerMultiplexerNode::isDesired(const std::string &node_name) const
{
    const auto &names = modes_[current_mode_idx_].controller_names;
    return std::find(names.begin(), names.end(), node_name) != names.end();
}

void ControllerMultiplexerNode::subscribeController(ControllerRuntime &rt)
{
    rt.sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        rt.cfg.topic_name, 10,
        [this, cfg = rt.cfg](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
        {
            const auto now = this->now();
            if (cfg.provides_lateral)
            {
                command_state_.steering_angle = msg->drive.steering_angle;
                command_state_.steering_timestamp = now;
            }
            if (cfg.provides_longitudinal)
            {
                command_state_.speed = msg->drive.speed;
                command_state_.speed_timestamp = now;
            }
        });
}

void ControllerMultiplexerNode::requestTransition(const std::string &node_name, uint8_t transition_id)
{
    auto &rt = controllers_.at(node_name);
    rt.transition_in_flight = true;

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    rt.client->async_send_request(
        request,
        [this, node_name, transition_id](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
        {
            // Node may have vanished between send and response (e.g. process killed).
            auto it = controllers_.find(node_name);
            if (it == controllers_.end())
                return;
            auto &rt = it->second;
            rt.transition_in_flight = false;

            const bool success = future.get() && future.get()->success;
            if (!success)
            {
                RCLCPP_WARN(this->get_logger(), "Transition %d on '%s' failed, will retry",
                            transition_id, node_name.c_str());
                return; // leave state as-is, next reconcile tick retries
            }

            using lifecycle_msgs::msg::Transition;
            if (transition_id == Transition::TRANSITION_CONFIGURE)
            {
                rt.state = LifecycleState::INACTIVE;
            }
            else if (transition_id == Transition::TRANSITION_ACTIVATE)
            {
                rt.state = LifecycleState::ACTIVE;
                subscribeController(rt);
                RCLCPP_INFO(this->get_logger(), "Controller '%s' active", node_name.c_str());
            }
            else if (transition_id == Transition::TRANSITION_DEACTIVATE)
            {
                rt.state = LifecycleState::INACTIVE;
                rt.sub.reset();
                RCLCPP_INFO(this->get_logger(), "Controller '%s' deactivated", node_name.c_str());
            }
        });
}

void ControllerMultiplexerNode::reconcileControllers()
{
    using lifecycle_msgs::msg::Transition;

    for (auto &[name, rt] : controllers_)
    {
        if (rt.transition_in_flight)
            continue; // wait for the in-flight response before deciding anything new

        const bool service_up = rt.client->service_is_ready();

        if (!service_up)
        {
            if (rt.state != LifecycleState::UNAVAILABLE)
            {
                RCLCPP_WARN(this->get_logger(), "Controller '%s' went offline", name.c_str());
                rt.state = LifecycleState::UNAVAILABLE;
                rt.sub.reset();
            }
            continue; // nothing to do until it reappears
        }

        if (rt.state == LifecycleState::UNAVAILABLE)
        {
            RCLCPP_INFO(this->get_logger(), "Controller '%s' came online", name.c_str());
            rt.state = LifecycleState::UNCONFIGURED;
        }

        const bool desired = isDesired(name);

        if (desired && rt.state == LifecycleState::UNCONFIGURED)
        {
            requestTransition(name, Transition::TRANSITION_CONFIGURE);
        }
        else if (desired && rt.state == LifecycleState::INACTIVE)
        {
            requestTransition(name, Transition::TRANSITION_ACTIVATE);
        }
        else if (!desired && rt.state == LifecycleState::ACTIVE)
        {
            requestTransition(name, Transition::TRANSITION_DEACTIVATE);
        }
    }
}

void ControllerMultiplexerNode::controlLoop()
{
    const auto now = this->now();
    ackermann_msgs::msg::AckermannDriveStamped output;
    output.header.stamp = now;
    output.header.frame_id = "base_link";
    output.drive.steering_angle = 0.0;
    output.drive.speed = 0.0;

    if (now - command_state_.speed_timestamp <= control_timeout_)
    {
        output.drive.speed = command_state_.speed;
    }
    if (now - command_state_.steering_timestamp <= control_timeout_)
    {
        output.drive.steering_angle = command_state_.steering_angle;
    }

    cmd_pub_->publish(output);
}

void ControllerMultiplexerNode::handleSwitchController(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    current_mode_idx_ = (current_mode_idx_ + 1) % modes_.size();
    command_state_ = CommandState{}; // drop stale commands from the previous controller(s)

    response->success = true;
    response->message = "Requested mode " + modes_[current_mode_idx_].name +
                        " — controllers will come up as they become available";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    // Actual activation/deactivation happens on the next reconcileControllers() tick.
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerMultiplexerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}