#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "ferrari_common/msg/control_cmd.hpp"
#include "ferrari_common/srv/get_gate_mode.hpp"
#include "ferrari_common/srv/set_gate_mode.hpp"
#include "ferrari_common/srv/toggle_arm_vehicle.hpp"

class ControlInterface : public rclcpp::Node
{
protected:
    rclcpp::Publisher<ferrari_common::msg::ControlCmd>::SharedPtr pub_external_control_cmd;
    rclcpp::Client<ferrari_common::srv::GetGateMode>::SharedPtr srv_get_gate_mode;
    rclcpp::Client<ferrari_common::srv::SetGateMode>::SharedPtr srv_set_gate_mode;
    rclcpp::Client<ferrari_common::srv::ToggleArmVehicle>::SharedPtr srv_toggle_arm_vehicle;

public:
    ControlInterface();
    ~ControlInterface();

    void handleSetGateMode();
    void handleToggleArmVehicle();
    void publishControlCmd(int16_t vehicle_yaw_rate,
                           int16_t vehicle_longitudinal_rate,
                           int16_t vehicle_lateral_rate,
                           bool horn);
};
