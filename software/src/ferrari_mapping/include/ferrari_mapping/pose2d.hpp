#pragma once

#include <cmath>

namespace ferrari_mapping
{
    struct Pose2D
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    inline double normalizeAngle(double angle)
    {
        return std::remainder(angle, 2.0 * M_PI);
    }

    inline Pose2D compose(const Pose2D &a, const Pose2D &b)
    {
        const double c = std::cos(a.yaw), s = std::sin(a.yaw);
        return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalizeAngle(a.yaw + b.yaw)};
    }

    inline Pose2D inverse(const Pose2D &a)
    {
        const double c = std::cos(a.yaw), s = std::sin(a.yaw);
        return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), normalizeAngle(-a.yaw)};
    }
}
