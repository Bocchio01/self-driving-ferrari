#pragma once

#include <nav_msgs/msg/odometry.h>

#include "network/network.hpp"
#include "network/virtuals/publisher.hpp"
#include "sensors/rotary_encoder.hpp"
#include "sensors/imu.hpp"

class PublisherOdom : public IPublisher
{
public:
    /**
     * @brief Construct a new Publisher Odom object
     * @param rate_hz The desired publishing frequency in Hertz (default 30 Hz)
     */
    PublisherOdom(float rate_hz = 30.0f)
    {
        publish_interval_ms_ = (unsigned long)(1000.0f / rate_hz);

        memset(&odom_msg_, 0, sizeof(nav_msgs__msg__Odometry));

        odom_msg_.header.frame_id.data = (char *)"odom";
        odom_msg_.header.frame_id.size = strlen("odom");
        odom_msg_.header.frame_id.capacity = odom_msg_.header.frame_id.size + 1;

        odom_msg_.child_frame_id.data = (char *)"base_link";
        odom_msg_.child_frame_id.size = strlen("base_link");
        odom_msg_.child_frame_id.capacity = odom_msg_.child_frame_id.size + 1;
    }
    /**
     * @brief Initialize the publisher in the micro-ROS network
     */
    void init(rcl_node_t *node) override
    {
        // Notice we use getPublisher() here because publisher_ is private in IPublisher
        RCCHECK(rclc_publisher_init_best_effort(
            getPublisher(),
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/vehicle/odom"));
    }

    /**
     * @brief Fetch data from sensors, construct the odometry message, and publish
     */
    void publish() override
    {
        unsigned long current_time = millis();
        if (current_time - last_publish_time_ < publish_interval_ms_)
        {
            return;
        }

        last_publish_time_ = current_time;

        vehicle::interfaces::Vehicle *vehicle = Network::getVehicle();
        vehicle::interfaces::OdometryState odom_state = vehicle->getOdometryState();

        int64_t time_ms = rmw_uros_epoch_millis();
        odom_msg_.header.stamp.sec = time_ms / 1000;
        odom_msg_.header.stamp.nanosec = (time_ms % 1000) * 1000000;

        // Position
        odom_msg_.pose.pose.position.x = odom_state.x;
        odom_msg_.pose.pose.position.y = odom_state.y;
        odom_msg_.pose.pose.position.z = 0.0;

        // Orientation
        odom_msg_.pose.pose.orientation.x = 0.0f;
        odom_msg_.pose.pose.orientation.y = 0.0f;
        odom_msg_.pose.pose.orientation.z = sinf(odom_state.theta / 2.0f);
        odom_msg_.pose.pose.orientation.w = cosf(odom_state.theta / 2.0f); // Use cosf instead of cos

        float magnitude = sqrtf((odom_msg_.pose.pose.orientation.x * odom_msg_.pose.pose.orientation.x) +
                                (odom_msg_.pose.pose.orientation.y * odom_msg_.pose.pose.orientation.y) +
                                (odom_msg_.pose.pose.orientation.z * odom_msg_.pose.pose.orientation.z) +
                                (odom_msg_.pose.pose.orientation.w * odom_msg_.pose.pose.orientation.w));

        if (magnitude < 1e-6f)
        {
            odom_msg_.pose.pose.orientation.x = 0.0f;
            odom_msg_.pose.pose.orientation.y = 0.0f;
            odom_msg_.pose.pose.orientation.z = 0.0f;
            odom_msg_.pose.pose.orientation.w = 1.0f;
        }
        else
        {
            // 4. Normalize components safely
            odom_msg_.pose.pose.orientation.x /= magnitude;
            odom_msg_.pose.pose.orientation.y /= magnitude;
            odom_msg_.pose.pose.orientation.z /= magnitude;
            odom_msg_.pose.pose.orientation.w /= magnitude;
        }

        // Twist / Velocity
        odom_msg_.twist.twist.linear.x = odom_state.linear_x;
        odom_msg_.twist.twist.angular.z = odom_state.angular_z;

        Serial.print("Publishing odometry message...");
        RCCHECK(rcl_publish(getPublisher(), &odom_msg_, NULL));
        Serial.println("done.");
    }

private:
    unsigned long last_publish_time_ = 0;
    unsigned long publish_interval_ms_;
    nav_msgs__msg__Odometry odom_msg_;
};