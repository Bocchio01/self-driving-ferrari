#pragma once

#include <as5600.hpp>
#include "sensors/sensor.hpp"

namespace sensors
{

    struct RotaryEncoderOutput
    {
        // Cumulative angle in radians
        int32_t cumulative_angle;

        // Angular speed in radians per second
        float angular_speed;
    };

    class RotaryEncoder : public interfaces::Sensor<RotaryEncoderOutput>, public AS5600
    {

    public:
        RotaryEncoder(TwoWire *i2c, uint8_t DIR = AS5600::NO_PIN, uint8_t OUT = AS5600::NO_PIN)
            : AS5600(i2c, DIR, OUT)
        {
        }

        void update()
        {
            AS5600::update();

            float angular_speed = getAngularSpeed(AS5600::SpeedUnit::RADIANS_PER_SECOND);
            if (!isnan(angular_speed))
            {
                // Exponential Moving Average (EMA) filter for angular speed
                static constexpr float alpha = 0.2f;
                filtered_angular_speed = alpha * angular_speed + (1.0f - alpha) * filtered_angular_speed;
            }
        }

        RotaryEncoderOutput data() override
        {
            int32_t cumulative_angle = getCumulativeAngle(AngleUnit::RADIANS);

            return {cumulative_angle, filtered_angular_speed};
        }

    private:
        float filtered_angular_speed = 0.0f;
    };

}