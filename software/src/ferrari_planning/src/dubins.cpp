#include "ferrari_planning/dubins.hpp"

#include <cmath>
#include <limits>

namespace ferrari_planning
{
    namespace
    {
        enum Segment
        {
            L,
            S,
            R
        };

        constexpr Segment kWords[6][3] = {{L, S, L}, {L, S, R}, {R, S, L}, {R, S, R}, {R, L, R}, {L, R, L}};

        double mod2pi(double angle)
        {
            double a = std::fmod(angle, 2.0 * M_PI);
            if (a < 0.0)
            {
                a += 2.0 * M_PI;
            }
            return a;
        }

        bool compute(int word, double alpha, double beta, double d, double &t, double &p, double &q)
        {
            const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
            const double c_ab = std::cos(alpha - beta);
            switch (word)
            {
            case 0:
            {
                const double p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb);
                if (p_sq < 0.0)
                {
                    return false;
                }
                const double tmp = std::atan2(cb - ca, d + sa - sb);
                t = mod2pi(-alpha + tmp);
                p = std::sqrt(p_sq);
                q = mod2pi(beta - tmp);
                return true;
            }
            case 3:
            {
                const double p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sb - sa);
                if (p_sq < 0.0)
                {
                    return false;
                }
                const double tmp = std::atan2(ca - cb, d - sa + sb);
                t = mod2pi(alpha - tmp);
                p = std::sqrt(p_sq);
                q = mod2pi(-beta + tmp);
                return true;
            }
            case 1:
            {
                const double p_sq = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb);
                if (p_sq < 0.0)
                {
                    return false;
                }
                p = std::sqrt(p_sq);
                const double tmp = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
                t = mod2pi(-alpha + tmp);
                q = mod2pi(-mod2pi(beta) + tmp);
                return true;
            }
            case 2:
            {
                const double p_sq = -2.0 + d * d + 2.0 * c_ab - 2.0 * d * (sa + sb);
                if (p_sq < 0.0)
                {
                    return false;
                }
                p = std::sqrt(p_sq);
                const double tmp = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
                t = mod2pi(alpha - tmp);
                q = mod2pi(beta - tmp);
                return true;
            }
            case 4:
            {
                const double tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
                if (std::abs(tmp) > 1.0)
                {
                    return false;
                }
                p = mod2pi(2.0 * M_PI - std::acos(tmp));
                t = mod2pi(alpha - std::atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0));
                q = mod2pi(alpha - beta - t + mod2pi(p));
                return true;
            }
            default:
            {
                const double tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (-sa + sb)) / 8.0;
                if (std::abs(tmp) > 1.0)
                {
                    return false;
                }
                p = mod2pi(2.0 * M_PI - std::acos(tmp));
                t = mod2pi(-alpha - std::atan2(ca - cb, d + sa - sb) + p / 2.0);
                q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
                return true;
            }
            }
        }

        PlannerPose advance(const PlannerPose &pose, Segment type, double length, double radius)
        {
            PlannerPose next = pose;
            if (type == S)
            {
                next.x += length * radius * std::cos(pose.yaw);
                next.y += length * radius * std::sin(pose.yaw);
            }
            else if (type == L)
            {
                next.x += radius * (std::sin(pose.yaw + length) - std::sin(pose.yaw));
                next.y += radius * (-std::cos(pose.yaw + length) + std::cos(pose.yaw));
                next.yaw = pose.yaw + length;
            }
            else
            {
                next.x += radius * (-std::sin(pose.yaw - length) + std::sin(pose.yaw));
                next.y += radius * (std::cos(pose.yaw - length) - std::cos(pose.yaw));
                next.yaw = pose.yaw - length;
            }
            return next;
        }

        PlannerPose endOf(const DubinsPath &path, const PlannerPose &start)
        {
            PlannerPose pose = start;
            const double lengths[3] = {path.t, path.p, path.q};
            for (int i = 0; i < 3; ++i)
            {
                pose = advance(pose, kWords[path.word][i], lengths[i], path.radius);
            }
            return pose;
        }
    }

    std::optional<DubinsPath> shortestDubinsPath(const PlannerPose &start, const PlannerPose &goal, double radius)
    {
        const double dx = goal.x - start.x, dy = goal.y - start.y;
        const double d = std::hypot(dx, dy) / radius;
        const double theta = mod2pi(std::atan2(dy, dx));
        const double alpha = mod2pi(start.yaw - theta);
        const double beta = mod2pi(goal.yaw - theta);

        std::optional<DubinsPath> best;
        double best_length = std::numeric_limits<double>::infinity();
        for (int word = 0; word < 6; ++word)
        {
            DubinsPath candidate;
            candidate.word = word;
            candidate.radius = radius;
            if (!compute(word, alpha, beta, d, candidate.t, candidate.p, candidate.q))
            {
                continue;
            }
            const PlannerPose end = endOf(candidate, start);
            const double yaw_error = std::abs(std::remainder(end.yaw - goal.yaw, 2.0 * M_PI));
            if (std::hypot(end.x - goal.x, end.y - goal.y) > 1e-3 || yaw_error > 1e-3)
            {
                continue;
            }
            if (candidate.length() < best_length)
            {
                best_length = candidate.length();
                best = candidate;
            }
        }
        return best;
    }

    std::vector<PlannerPose> sampleDubinsPath(const DubinsPath &path, const PlannerPose &start, double spacing)
    {
        std::vector<PlannerPose> samples;
        PlannerPose pose = start;
        const double lengths[3] = {path.t, path.p, path.q};
        for (int i = 0; i < 3; ++i)
        {
            const double segment_length = lengths[i] * path.radius;
            const int pieces = std::max(1, static_cast<int>(std::ceil(segment_length / spacing)));
            for (int k = 1; k <= pieces; ++k)
            {
                PlannerPose sample = advance(pose, kWords[path.word][i], lengths[i] * k / pieces, path.radius);
                sample.yaw = std::remainder(sample.yaw, 2.0 * M_PI);
                samples.push_back(sample);
            }
            pose = advance(pose, kWords[path.word][i], lengths[i], path.radius);
        }
        return samples;
    }
}
