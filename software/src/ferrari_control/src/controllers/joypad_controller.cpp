#include <functional>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "ferrari_control/controllers/joypad_controller.hpp"

using std::placeholders::_1;

JoypadControllerNode::JoypadControllerNode(const rclcpp::NodeOptions &options)
    : Controller("joypad_cmd", "joypad_controller_node", options),
      toggle_actuators_prev_(false),
      switch_controller_prev_(false)
{
    steering_axis_ = this->declare_parameter<int>("steering_axis", 0);
    speed_axis_ = this->declare_parameter<int>("speed_axis", 1);
    toggle_actuators_button_ = this->declare_parameter<int>("toggle_actuators_button", 7);
    switch_controller_button_ = this->declare_parameter<int>("switch_controller_button", 6);
    invert_steering_ = this->declare_parameter<bool>("invert_steering", false);
    invert_speed_ = this->declare_parameter<bool>("invert_speed", false);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 20, std::bind(&JoypadControllerNode::joyCallback, this, _1));
    toggle_actuators_client_ = this->create_client<std_srvs::srv::Trigger>("toggle_actuators");
    switch_controller_client_ = this->create_client<std_srvs::srv::Trigger>("switch_controller");

    RCLCPP_INFO(this->get_logger(), "joypad_controller_node booted. Waiting to be configured by multiplexer.");
}

void JoypadControllerNode::computeControlCommand()
{
    // This function is intentionally left empty because the control command is computed in the joyCallback.
}

void JoypadControllerNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    const bool toggle_actuators_pressed = readButton(*msg, toggle_actuators_button_);
    if (toggle_actuators_pressed && !toggle_actuators_prev_)
    {
        requestToggleActuators();
    }
    toggle_actuators_prev_ = toggle_actuators_pressed;

    const bool switch_controller_pressed = readButton(*msg, switch_controller_button_);
    if (switch_controller_pressed && !switch_controller_prev_)
    {
        requestSwitchController();
    }
    switch_controller_prev_ = switch_controller_pressed;

    if (!is_active_)
        return;

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.drive.steering_angle = (invert_steering_ ? -1.0 : 1.0) * this->max_steering_angle_ * readAxis(*msg, steering_axis_);
    cmd.drive.speed = (invert_speed_ ? -1.0 : 1.0) * this->max_speed_ * readAxis(*msg, speed_axis_);

    publishCmd(cmd);
}

double JoypadControllerNode::readAxis(const sensor_msgs::msg::Joy &msg, int axis_index) const
{
    if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg.axes.size())
        return 0.0;
    return std::clamp(static_cast<double>(msg.axes[static_cast<size_t>(axis_index)]), -1.0, 1.0);
}

bool JoypadControllerNode::readButton(const sensor_msgs::msg::Joy &msg, int button_index) const
{
    if (button_index < 0 || static_cast<size_t>(button_index) >= msg.buttons.size())
        return false;
    return msg.buttons[static_cast<size_t>(button_index)] != 0;
}

void JoypadControllerNode::requestToggleActuators() const
{
    if (!toggle_actuators_client_->wait_for_service(std::chrono::milliseconds(10)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Could not toggle actuators: service /toggle_actuators not available");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    toggle_actuators_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            try
            {
                if (future.get()->success)
                    RCLCPP_INFO(this->get_logger(), "Successfully toggled vehicle engagement");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
            }
        });
}

void JoypadControllerNode::requestSwitchController() const
{
    if (!switch_controller_client_->wait_for_service(std::chrono::milliseconds(10)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Could not switch controller: service /switch_controller not available");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    switch_controller_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            try
            {
                if (future.get()->success)
                    RCLCPP_INFO(this->get_logger(), "Successfully switched controller");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
            }
        });
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<JoypadControllerNode> node = std::make_shared<JoypadControllerNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}