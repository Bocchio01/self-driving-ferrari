#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "ferrari_common/msg/control_cmd.hpp"
#include "ferrari_common/srv/get_gate_mode.hpp"
#include "ferrari_common/srv/set_gate_mode.hpp"
#include "ferrari_common/srv/toggle_arm_vehicle.hpp"
#include "control_interface.hpp"

ControlInterface::ControlInterface()
    : Node("control_interface")
{
    this->pub_external_control_cmd = this->create_publisher<ferrari_common::msg::ControlCmd>("external_control_cmd", 10);
    this->srv_get_gate_mode = this->create_client<ferrari_common::srv::GetGateMode>("get_gate_mode");
    this->srv_set_gate_mode = this->create_client<ferrari_common::srv::SetGateMode>("set_gate_mode");
    this->srv_toggle_arm_vehicle = this->create_client<ferrari_common::srv::ToggleArmVehicle>("toggle_arm_vehicle");
}

ControlInterface::~ControlInterface() {}

void ControlInterface::handleSetGateMode()
{
    auto request_get = std::make_shared<ferrari_common::srv::GetGateMode::Request>();
    auto request_set = std::make_shared<ferrari_common::srv::SetGateMode::Request>();

    // Call get gate mode service
    while (!srv_get_gate_mode->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "Waiting for 'get_gate_mode' service to be available...");
    }
    auto result_get = srv_get_gate_mode->async_send_request(request_get);

    // Once response is available
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_get) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto gate_mode = result_get.get()->gate_mode;
        request_set->gate_mode = (gate_mode + 1) % 2;

        // Call set gate mode service
        while (!srv_set_gate_mode->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Waiting for 'set_gate_mode' service to be available...");
        }
        auto result_set = srv_set_gate_mode->async_send_request(request_set);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_set) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (!result_set.get()->success)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to change the gate mode");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Gate mode changed to %s", request_set->gate_mode == 0 ? "internal" : "external");
            }
        }
    }
}

void ControlInterface::handleToggleArmVehicle()
{
    auto request = std::make_shared<ferrari_common::srv::ToggleArmVehicle::Request>();

    // Call toggle arm vehicle service
    while (!srv_toggle_arm_vehicle->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "Waiting for 'toggle_arm_vehicle' service to be available...");
    }
    auto result = srv_toggle_arm_vehicle->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (!result.get()->success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm/disarm the vehicle");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Vehicle %s", result.get()->is_armed ? "armed" : "disarmed");
        }
    }
}

void ControlInterface::publishControlCmd(int16_t vehicle_yaw_rate, int16_t vehicle_longitudinal_rate, int16_t vehicle_lateral_rate, bool horn)
{
    auto control_cmd = std::make_shared<ferrari_common::msg::ControlCmd>();

    control_cmd->motion_cmd.vehicle_yaw_rate = vehicle_yaw_rate;
    control_cmd->motion_cmd.vehicle_longitudinal_rate = vehicle_longitudinal_rate;
    control_cmd->motion_cmd.vehicle_lateral_rate = vehicle_lateral_rate;
    control_cmd->horn = horn;

    pub_external_control_cmd->publish(*control_cmd);
}
