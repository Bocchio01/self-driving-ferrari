#include <ros/ros.h>
#include "vehicle_cmd_gate.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_cmd_gate");
    VehicleCmdGate vehicle_cmd_gate;
    ros::spin();
    return 0;
}
