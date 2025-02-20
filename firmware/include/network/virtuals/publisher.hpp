#pragma once

#include <ros.h>

class Publisher {
public:
    virtual ~Publisher() {}
    virtual void init(ros::NodeHandle& nh) = 0;
    virtual void publish() = 0;
};
