#pragma once

#include <sensor_msgs/msg/imu.h>

#include "network/network.hpp"
#include "network/virtuals/publisher.hpp"
#include "sensors/imu.hpp"

class PublisherImu : public IPublisher
{
public:
    PublisherImu(sensors::IMU &imu, float rate_hz = 50.0f)
        : imu_(imu)
    {
        publish_interval_ms_ = (unsigned long)(1000.0f / rate_hz);

        memset(&imu_msg_, 0, sizeof(sensor_msgs__msg__Imu));

        imu_msg_.header.frame_id.data = (char *)"imu_link";
        imu_msg_.header.frame_id.size = strlen("imu_link");
        imu_msg_.header.frame_id.capacity = imu_msg_.header.frame_id.size + 1;

        imu_msg_.orientation_covariance[0] = -1.0;

        for (uint8_t i = 0; i < 3; ++i)
        {
            imu_msg_.angular_velocity_covariance[i * 4] = GYRO_VARIANCE;
            imu_msg_.linear_acceleration_covariance[i * 4] = ACCEL_VARIANCE;
        }
    }

    void init(rcl_node_t *node) override
    {
        RCCHECK(rclc_publisher_init_best_effort(
            getPublisher(),
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/vehicle/imu_raw"));
    }

    void publish() override
    {
        unsigned long current_time = millis();
        if (current_time - last_publish_time_ < publish_interval_ms_)
        {
            return;
        }

        last_publish_time_ = current_time;

        noInterrupts();
        sensors::IMUOutput data = imu_.data();
        interrupts();

        int64_t time_ms = rmw_uros_epoch_millis();
        imu_msg_.header.stamp.sec = time_ms / 1000;
        imu_msg_.header.stamp.nanosec = (time_ms % 1000) * 1000000;

        imu_msg_.angular_velocity.x = data.gyro_x * DEG_TO_RAD_F;
        imu_msg_.angular_velocity.y = data.gyro_y * DEG_TO_RAD_F;
        imu_msg_.angular_velocity.z = data.gyro_z * DEG_TO_RAD_F;

        imu_msg_.linear_acceleration.x = data.accel_x * STANDARD_GRAVITY;
        imu_msg_.linear_acceleration.y = data.accel_y * STANDARD_GRAVITY;
        imu_msg_.linear_acceleration.z = data.accel_z * STANDARD_GRAVITY;

        RCCHECK(rcl_publish(getPublisher(), &imu_msg_, NULL));
    }

private:
    static constexpr float DEG_TO_RAD_F = PI / 180.0f;
    static constexpr float STANDARD_GRAVITY = 9.80665f;
    static constexpr double GYRO_VARIANCE = 1.0e-4;
    static constexpr double ACCEL_VARIANCE = 2.5e-3;

    sensors::IMU &imu_;
    unsigned long last_publish_time_ = 0;
    unsigned long publish_interval_ms_;
    sensor_msgs__msg__Imu imu_msg_;
};
