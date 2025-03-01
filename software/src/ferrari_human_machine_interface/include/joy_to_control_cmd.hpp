#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "ferrari_common/control_cmd.h"

class JoyToControlCmd
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_joy;
    ros::Publisher pub_external_control_cmd;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

public:
    JoyToControlCmd();
    ~JoyToControlCmd();
};
