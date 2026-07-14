#pragma once

#include <cstddef>
#include <stdint.h>
#include <Arduino.h>

// Here we make the hypothesis that the vehicle is <Ackermann> type
namespace vehicle::configs
{
    template <size_t N, typename X, typename Y>
    struct LookupTable
    {
        struct Point
        {
            X x;
            Y y;
        };

        Point points[N];

        constexpr Y operator()(X x_star) const
        {
            if (x_star < points[0].x)
                return points[0].y;

            if (x_star > points[N - 1].x)
                return points[N - 1].y;

            for (size_t i = 0; i < N - 1; i++)
            {
                if (x_star <= points[i + 1].x)
                {
                    X x0 = points[i].x;
                    Y y0 = points[i].y;
                    X x1 = points[i + 1].x;
                    Y y1 = points[i + 1].y;

                    if (x1 == x0)
                        return y0;

                    return static_cast<Y>(y0 + (y1 - y0) * (x_star - x0) / (x1 - x0));
                }
            }

            // Fallback return value, should never be reached
            return static_cast<Y>(0);
        }
    };

    struct Kinematic
    {
        static constexpr float WHEELBASE = 0.218;                          // [m]
        static constexpr float TRACK_WIDTH = 0.135;                        // [m]
        static constexpr float BELT_RATIO = 0.75f;                         // encoder:wheel rotation ratio
        static constexpr float WHEEL_RADIUS = 0.0575f / 2.0f;              // [m]
        static constexpr float MAX_SPEED = 1.00f;                          // [m/s]
        static constexpr float MAX_STEERING_ANGLE = 22.5f * (PI / 180.0f); // [rad]
        static constexpr float MAX_ACCELERATIONS[3] = {1.0f, 3.0f, 9.0f};  // [m/s^2]
    };

    struct Actuators
    {
        static constexpr float LOOP_RATE = 50.0f; // [Hz] Loop update rate

        // Propulsion actuator configuration
        static constexpr LookupTable<29, float, int16_t> ANGULAR_VELOCITY_TO_MOTOR_PWM = { // [m/s] -> [-255-+255]
            {{-3.26f / Kinematic::WHEEL_RADIUS, -150},
             {-2.90f / Kinematic::WHEEL_RADIUS, -140},
             {-2.65f / Kinematic::WHEEL_RADIUS, -130},
             {-2.46f / Kinematic::WHEEL_RADIUS, -120},
             {-2.33f / Kinematic::WHEEL_RADIUS, -110},
             {-2.15f / Kinematic::WHEEL_RADIUS, -100},
             {-1.90f / Kinematic::WHEEL_RADIUS, -90},
             {-1.62f / Kinematic::WHEEL_RADIUS, -80},
             {-1.38f / Kinematic::WHEEL_RADIUS, -70},
             {-1.19f / Kinematic::WHEEL_RADIUS, -60},
             {-0.88f / Kinematic::WHEEL_RADIUS, -50},
             {-0.71f / Kinematic::WHEEL_RADIUS, -40},
             {-0.50f / Kinematic::WHEEL_RADIUS, -30},
             {-0.27f / Kinematic::WHEEL_RADIUS, -20},
             //  {+0.00f / Kinematic::WHEEL_RADIUS, -10},
             {+0.00f / Kinematic::WHEEL_RADIUS, +0},
             //  {+0.00f / Kinematic::WHEEL_RADIUS, +10},
             {+0.27f / Kinematic::WHEEL_RADIUS, +20},
             {+0.50f / Kinematic::WHEEL_RADIUS, +30},
             {+0.71f / Kinematic::WHEEL_RADIUS, +40},
             {+0.88f / Kinematic::WHEEL_RADIUS, +50},
             {+1.19f / Kinematic::WHEEL_RADIUS, +60},
             {+1.38f / Kinematic::WHEEL_RADIUS, +70},
             {+1.62f / Kinematic::WHEEL_RADIUS, +80},
             {+1.90f / Kinematic::WHEEL_RADIUS, +90},
             {+2.15f / Kinematic::WHEEL_RADIUS, +100},
             {+2.33f / Kinematic::WHEEL_RADIUS, +110},
             {+2.46f / Kinematic::WHEEL_RADIUS, +120},
             {+2.65f / Kinematic::WHEEL_RADIUS, +130},
             {+2.90f / Kinematic::WHEEL_RADIUS, +140},
             {+3.26f / Kinematic::WHEEL_RADIUS, +150}}};
        static constexpr int16_t MAX_MOTOR_PWM = ANGULAR_VELOCITY_TO_MOTOR_PWM(+Kinematic::MAX_SPEED / Kinematic::WHEEL_RADIUS); // [-255-+255]
        static constexpr int16_t MIN_MOTOR_PWM = ANGULAR_VELOCITY_TO_MOTOR_PWM(-Kinematic::MAX_SPEED / Kinematic::WHEEL_RADIUS); // [-255-+255] Maybe put a specific reverse speed limit here, if needed

        // Steering actuator configuration
        static constexpr LookupTable<3, float, uint8_t> STEERING_ANGLE_TO_SERVO_PWM = { // [rad] -> [0-255]
            {{-22.5f * (PI / 180.0f), 35},
             {+0.00f * (PI / 180.0f), 75},
             {+22.5f * (PI / 180.0f), 115}}};
        static constexpr float MAX_STEERING_ANGLE = STEERING_ANGLE_TO_SERVO_PWM(+Kinematic::MAX_STEERING_ANGLE); // [rad]
        static constexpr float MIN_STEERING_ANGLE = STEERING_ANGLE_TO_SERVO_PWM(-Kinematic::MAX_STEERING_ANGLE); // [rad]
    };

}