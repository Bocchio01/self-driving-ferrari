#pragma once

#include <mpu6050.hpp>
#include "sensors/sensor.hpp"

namespace sensors
{
    struct IMUOutput
    {
        float accel_x; // g (Longitudinal)
        float accel_y; // g (Lateral)
        float accel_z; // g (Vertical)
        float gyro_x;  // deg/s (Roll)
        float gyro_y;  // deg/s (Pitch)
        float gyro_z;  // deg/s (Yaw)
    };

    class IMU : public interfaces::Sensor<IMUOutput>, public MPU6050
    {
    public:
        IMU(TwoWire *i2c, uint8_t address = MPU6050::DEFAULT_ADDRESS)
            : MPU6050(i2c, address) {}

        void update()
        {
            MPU6050::update();

            accel = getAcceleration();
            gyro = getGyroscope();
        }

        IMUOutput data() override
        {
            return {accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z};
        }

    private:
        MPU6050::Axis3<float> accel;
        MPU6050::Axis3<float> gyro;
    };
}