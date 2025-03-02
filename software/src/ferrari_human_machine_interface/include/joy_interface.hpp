#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "ferrari_common/control_cmd.h"
#include "control_interface.hpp"

enum class JoyButtons
{
    TRIANGLE,
    CIRCLE,
    CROSS,
    SQUARE,
    L1,
    R1,
    L2,
    R2,
    SELECT,
    START,
    LEFT_STICK_Z,
    RIGHT_STICK_Z
};

enum class JoyAxes
{
    LEFT_STICK_X,
    LEFT_STICK_Y,
    RIGHT_STICK_X,
    RIGHT_STICK_Y
};

class JoyInterface : public ControlInterface
{
private:
    ros::Subscriber sub_joy;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

public:
    JoyInterface();
    ~JoyInterface();
};
