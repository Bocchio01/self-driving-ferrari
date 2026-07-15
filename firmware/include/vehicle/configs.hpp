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

        String toString(const char *name = "Lookup Table") const
        {
            String out = String(name) + ":\n";
            for (size_t i = 0; i < N; i++)
            {
                out += String(i, 2) + ": x = " + String(points[i].x) + ", y = " + String(points[i].y) + "\n";
            }
            return out;
        }
    };

    struct Kinematic
    {
        static constexpr float WHEELBASE = 0.218;                          // [m]
        static constexpr float TRACK_WIDTH = 0.135;                        // [m]
        static constexpr float BELT_RATIO = 0.75f;                         // encoder:wheel rotation ratio
        static constexpr float WHEEL_RADIUS = 0.0575f / 2.0f;              // [m]
        static constexpr float MAX_SPEED = 1.00f;                          // [m/s]
        static constexpr float MAX_STEERING_ANGLE = 20.0f * (PI / 180.0f); // [rad]
        static constexpr float MAX_ACCELERATIONS[3] = {1.0f, 3.0f, 9.0f};  // [m/s^2]
    };

    struct Actuators
    {
        static constexpr float LOOP_RATE = 50.0f; // [Hz] Loop update rate

        // Propulsion actuator configuration
        static constexpr LookupTable<42, float, int16_t> ANGULAR_VELOCITY_TO_MOTOR_PWM = { // [m/s] -> [-255-+255]
            {
                // Reverse direction
                {-2.782f / Kinematic::WHEEL_RADIUS, -150},
                {-2.736f / Kinematic::WHEEL_RADIUS, -140},
                {-2.449f / Kinematic::WHEEL_RADIUS, -130},
                {-2.362f / Kinematic::WHEEL_RADIUS, -120},
                {-2.135f / Kinematic::WHEEL_RADIUS, -110},
                {-1.937f / Kinematic::WHEEL_RADIUS, -100},
                {-1.730f / Kinematic::WHEEL_RADIUS, -90},
                {-1.559f / Kinematic::WHEEL_RADIUS, -80},
                {-1.312f / Kinematic::WHEEL_RADIUS, -70},
                {-1.092f / Kinematic::WHEEL_RADIUS, -60},
                {-0.952f / Kinematic::WHEEL_RADIUS, -50},
                {-0.693f / Kinematic::WHEEL_RADIUS, -40},
                {-0.525f / Kinematic::WHEEL_RADIUS, -30},
                {-0.323f / Kinematic::WHEEL_RADIUS, -20},
                {-0.115f / Kinematic::WHEEL_RADIUS, -10},

                // Rest
                {+0.000f / Kinematic::WHEEL_RADIUS, +0},

                // Forward direction
                {+0.141f / Kinematic::WHEEL_RADIUS, +10},
                {+0.315f / Kinematic::WHEEL_RADIUS, +20},
                {+0.543f / Kinematic::WHEEL_RADIUS, +30},
                {+0.742f / Kinematic::WHEEL_RADIUS, +40},
                {+0.934f / Kinematic::WHEEL_RADIUS, +50},
                {+0.998f / Kinematic::WHEEL_RADIUS, +60},
                {+1.370f / Kinematic::WHEEL_RADIUS, +70},
                {+1.546f / Kinematic::WHEEL_RADIUS, +80},
                {+1.760f / Kinematic::WHEEL_RADIUS, +90},
                {+1.541f / Kinematic::WHEEL_RADIUS, +100},
                {+2.083f / Kinematic::WHEEL_RADIUS, +110},
                {+2.352f / Kinematic::WHEEL_RADIUS, +120},
                {+2.367f / Kinematic::WHEEL_RADIUS, +130},
                {+2.707f / Kinematic::WHEEL_RADIUS, +140},
                {+2.984f / Kinematic::WHEEL_RADIUS, +150},
                {+2.981f / Kinematic::WHEEL_RADIUS, +160},
                {+3.042f / Kinematic::WHEEL_RADIUS, +170},
                {+3.174f / Kinematic::WHEEL_RADIUS, +180},
                {+3.506f / Kinematic::WHEEL_RADIUS, +190},
                {+3.670f / Kinematic::WHEEL_RADIUS, +200},
                {+3.764f / Kinematic::WHEEL_RADIUS, +210},
                {+3.619f / Kinematic::WHEEL_RADIUS, +220},
                {+3.963f / Kinematic::WHEEL_RADIUS, +230},
                {+3.666f / Kinematic::WHEEL_RADIUS, +240},
                {+4.185f / Kinematic::WHEEL_RADIUS, +250},
                {+4.141f / Kinematic::WHEEL_RADIUS, +255},
            }};
        static constexpr int16_t MAX_MOTOR_PWM = ANGULAR_VELOCITY_TO_MOTOR_PWM(+Kinematic::MAX_SPEED / Kinematic::WHEEL_RADIUS); // [-255-+255]
        static constexpr int16_t MIN_MOTOR_PWM = ANGULAR_VELOCITY_TO_MOTOR_PWM(-Kinematic::MAX_SPEED / Kinematic::WHEEL_RADIUS); // [-255-+255] Maybe put a specific reverse speed limit here, if needed

        // Steering actuator configuration
        static constexpr LookupTable<11, float, float> STEERING_ANGLE_TO_SERVO_PWM = { // [rad] -> float[0-255]
            {
                {-20.29464f * PI / 180.0f, 035.00000f},
                {-20.00000f * PI / 180.0f, 036.70929f}, // Computed from retro fitting the steering mechanism geometry
                {-18.58060f * PI / 180.0f, 040.00000f},
                {-13.42217f * PI / 180.0f, 050.00000f},
                {-08.11960f * PI / 180.0f, 060.00000f},
                {+00.00000f * PI / 180.0f, 074.84799f}, // Computed from retro fitting the steering mechanism geometry
                {+08.07010f * PI / 180.0f, 090.00000f},
                {+13.72617f * PI / 180.0f, 100.00000f},
                {+18.03609f * PI / 180.0f, 110.00000f},
                {+20.00000f * PI / 180.0f, 113.79745f}, // Computed from retro fitting the steering mechanism geometry
                {+20.16892f * PI / 180.0f, 115.00000f},
            }};
        static constexpr float MAX_STEERING_ANGLE = STEERING_ANGLE_TO_SERVO_PWM(+Kinematic::MAX_STEERING_ANGLE); // [rad]
        static constexpr float MIN_STEERING_ANGLE = STEERING_ANGLE_TO_SERVO_PWM(-Kinematic::MAX_STEERING_ANGLE); // [rad]
    };

};