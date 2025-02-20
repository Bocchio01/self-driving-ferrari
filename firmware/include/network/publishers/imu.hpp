#pragma once

#include <ros.h>
#include <sensor_msgs/Imu.h>

class IMUPublisher {
private:
    ros::Publisher pub;
    sensor_msgs::Imu imu_msg;

public:
    IMUPublisher() : pub("imu", &imu_msg) {}

    void init(ros::NodeHandle& nh) {
        nh.advertise(pub);
    }

    void publish(float x, float y, float z) {
        imu_msg.linear_acceleration.x = x;
        imu_msg.linear_acceleration.y = y;
        imu_msg.linear_acceleration.z = z;
        pub.publish(&imu_msg);
    }
};
