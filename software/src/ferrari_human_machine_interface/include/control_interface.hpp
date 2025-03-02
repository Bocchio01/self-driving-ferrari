#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "ferrari_common/control_cmd.h"
#include "ferrari_common/get_gate_mode.h"
#include "ferrari_common/set_gate_mode.h"
#include "ferrari_common/toggle_arm_vehicle.h"

class ControlInterface
{
protected:
    ros::NodeHandle nh;
    ros::Publisher pub_external_control_cmd;
    ros::ServiceClient srv_get_gate_mode;
    ros::ServiceClient srv_set_gate_mode;
    ros::ServiceClient srv_toggle_arm_vehicle;

public:
    ControlInterface();
    ~ControlInterface();

    void handleSetGateMode();
    void handleToggleArmVehicle();
    void publishControlCmd(float vehicle_yaw_rate,
                           float vehicle_longitudinal_rate,
                           float vehicle_lateral_rate,
                           bool horn);
};
