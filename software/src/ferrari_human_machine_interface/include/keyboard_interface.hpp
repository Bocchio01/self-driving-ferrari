#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "ferrari_common/control_cmd.h"
#include "control_interface.hpp"

enum class ArrowKeys
{
    UP = 65,
    DOWN,
    RIGHT,
    LEFT
};

class KeyboardInterface : public ControlInterface
{
private:
    int16_t steering;
    int16_t throttle;
    bool horn;

    void reset();

public:
    KeyboardInterface();
    ~KeyboardInterface();

    void run();
};
