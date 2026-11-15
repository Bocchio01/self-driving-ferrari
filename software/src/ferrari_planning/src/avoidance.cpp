#include "ferrari_planning/avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ferrari_planning
{
    namespace
    {
        double smoothstep(double t)
        {
            t = std::clamp(t, 0.0, 1.0);
            return t * t * (3.0 - 2.0 * t);
        }

        std::vector<PlannerPose> shiftedPath(const std::vector<PlannerPose> &base, const std::vector<double> &arc, double target, double start_offset, double ramp)
        {
            const double length = arc.back();
            const double effective_ramp = std::min(ramp, 0.5 * length);
            std::vector<PlannerPose> path(base.size());
            for (size_t i = 0; i < base.size(); ++i)
            {
                double offset = start_offset + (target - start_offset) * smoothstep(arc[i] / std::max(effective_ramp, 1e-6));
                offset *= 1.0 - smoothstep((arc[i] - (length - effective_ramp)) / std::max(effective_ramp, 1e-6));
                path[i] = {base[i].x - std::sin(base[i].yaw) * offset, base[i].y + std::cos(base[i].yaw) * offset, base[i].yaw};
            }
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                path[i].yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
            }
            if (path.size() > 1)
            {
                path.back().yaw = path[path.size() - 2].yaw;
            }
            return path;
        }

        bool evaluate(const std::vector<PlannerPose> &path, const std::vector<double> &arc, const CostGrid &grid, const AvoidanceParams &params, double &cost)
        {
            cost = 0.0;
            for (size_t i = 0; i < path.size(); ++i)
            {
                const double c = std::cos(path[i].yaw), s = std::sin(path[i].yaw);
                int worst = 0;
                for (double offset : params.footprint_offsets)
                {
                    const int value = grid.cost(path[i].x + c * offset, path[i].y + s * offset);
                    if (value >= params.lethal_threshold)
                    {
                        return false;
                    }
                    worst = std::max(worst, value);
                }
                const double spacing = i > 0 ? arc[i] - arc[i - 1] : 0.0;
                cost += params.cost_weight * worst / 100.0 * spacing;
            }
            return true;
        }
    }

    int maxPathCost(const std::vector<PlannerPose> &path, const CostGrid &grid, const AvoidanceParams &params)
    {
        int worst = 0;
        for (const auto &pose : path)
        {
            const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
            for (double offset : params.footprint_offsets)
            {
                worst = std::max(worst, grid.cost(pose.x + c * offset, pose.y + s * offset));
            }
        }
        return worst;
    }

    size_t firstCollisionIndex(const std::vector<PlannerPose> &path, const CostGrid &grid, const AvoidanceParams &params)
    {
        for (size_t i = 0; i < path.size(); ++i)
        {
            const double c = std::cos(path[i].yaw), s = std::sin(path[i].yaw);
            for (double offset : params.footprint_offsets)
            {
                if (grid.cost(path[i].x + c * offset, path[i].y + s * offset) >= params.lethal_threshold)
                {
                    return i;
                }
            }
        }
        return path.size();
    }

    AvoidanceResult planAvoidance(const std::vector<PlannerPose> &base, const CostGrid &grid, const AvoidanceParams &params, double start_offset, double previous_offset)
    {
        AvoidanceResult best;
        best.cost = std::numeric_limits<double>::infinity();
        if (base.size() < 2 || !grid.valid())
        {
            best.feasible = !base.empty();
            best.path = base;
            return best;
        }

        std::vector<double> arc(base.size(), 0.0);
        for (size_t i = 1; i < base.size(); ++i)
        {
            arc[i] = arc[i - 1] + std::hypot(base[i].x - base[i - 1].x, base[i].y - base[i - 1].y);
        }
        const double length = arc.back();

        const int steps = static_cast<int>(std::round(params.max_offset / params.offset_step));
        for (int k = -steps; k <= steps; ++k)
        {
            const double target = k * params.offset_step;
            const auto path = shiftedPath(base, arc, target, start_offset, params.ramp_length);
            double cost;
            if (!evaluate(path, arc, grid, params, cost))
            {
                continue;
            }
            cost += params.deviation_weight * std::abs(target) * length + params.change_weight * std::abs(target - previous_offset) +
                    params.start_offset_weight * std::abs(target - start_offset);
            if (cost < best.cost)
            {
                best.cost = cost;
                best.offset = target;
                best.path = path;
                best.feasible = true;
            }
        }
        if (!best.feasible)
        {
            best.path.clear();
        }
        return best;
    }
}
