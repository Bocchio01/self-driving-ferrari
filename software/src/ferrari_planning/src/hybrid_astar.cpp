#include "ferrari_planning/hybrid_astar.hpp"

#include "ferrari_planning/dubins.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace ferrari_planning
{
    namespace
    {
        double normalize(double angle)
        {
            return std::remainder(angle, 2.0 * M_PI);
        }

        bool footprintCollides(const CostGrid &grid, const PlannerPose &pose, const HybridAStarParams &params)
        {
            const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
            for (double offset : params.footprint_offsets)
            {
                const int cost = grid.cost(pose.x + c * offset, pose.y + s * offset);
                if (cost < 0 || cost >= params.lethal_threshold)
                {
                    return true;
                }
            }
            return false;
        }

        std::vector<float> distanceToGoal(const CostGrid &grid, const PlannerPose &goal, const HybridAStarParams &params)
        {
            const size_t total = static_cast<size_t>(grid.width) * grid.height;
            std::vector<float> distance(total, std::numeric_limits<float>::infinity());
            int gx, gy;
            if (!grid.toCell(goal.x, goal.y, gx, gy))
            {
                return distance;
            }

            using Entry = std::pair<float, int>;
            std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
            distance[static_cast<size_t>(gy) * grid.width + gx] = 0.0f;
            open.emplace(0.0f, gy * grid.width + gx);

            while (!open.empty())
            {
                const auto [d, index] = open.top();
                open.pop();
                if (d > distance[index])
                {
                    continue;
                }
                const int x = index % grid.width, y = index / grid.width;
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0)
                        {
                            continue;
                        }
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height)
                        {
                            continue;
                        }
                        const int cost = grid.data[static_cast<size_t>(ny) * grid.width + nx];
                        if (cost >= params.lethal_threshold)
                        {
                            continue;
                        }
                        const float step = static_cast<float>(grid.resolution * ((dx != 0 && dy != 0) ? M_SQRT2 : 1.0) * (1.0 + params.cost_weight * std::max(cost, 0) / 100.0));
                        const int next = ny * grid.width + nx;
                        if (d + step < distance[next])
                        {
                            distance[next] = d + step;
                            open.emplace(d + step, next);
                        }
                    }
                }
            }
            return distance;
        }

        struct Node
        {
            PlannerPose pose;
            double g;
            int parent;
            double curvature;
        };

        long long stateKey(const PlannerPose &pose, const HybridAStarParams &params)
        {
            const long long ix = static_cast<long long>(std::floor(pose.x / params.xy_resolution));
            const long long iy = static_cast<long long>(std::floor(pose.y / params.xy_resolution));
            long long it = static_cast<long long>(std::floor((normalize(pose.yaw) + M_PI) / (2.0 * M_PI) * params.yaw_bins));
            it = std::clamp<long long>(it, 0, params.yaw_bins - 1);
            return ((ix + 100000) * 200003LL + (iy + 100000)) * 128LL + it;
        }
    }

    PlanResult planHybridAStar(const CostGrid &grid, const PlannerPose &start, const PlannerPose &goal, const HybridAStarParams &params)
    {
        PlanResult result;
        if (!grid.valid() || footprintCollides(grid, goal, params))
        {
            return result;
        }

        const std::vector<float> h_field = distanceToGoal(grid, goal, params);
        auto heuristic = [&](const PlannerPose &p)
        {
            int ix, iy;
            if (!grid.toCell(p.x, p.y, ix, iy))
            {
                return std::numeric_limits<double>::infinity();
            }
            return static_cast<double>(h_field[static_cast<size_t>(iy) * grid.width + ix]);
        };

        std::vector<double> curvatures;
        const int n = params.steering_samples_per_side;
        for (int i = -n; i <= n; ++i)
        {
            curvatures.push_back(std::tan(params.max_steering_angle * params.steering_margin * i / std::max(n, 1)) / params.wheelbase);
        }

        std::vector<Node> nodes;
        nodes.push_back({start, 0.0, -1, 0.0});
        using Entry = std::pair<double, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        std::unordered_map<long long, double> best;
        open.emplace(params.heuristic_weight * std::max(heuristic(start), 0.0), 0);
        best[stateKey(start, params)] = 0.0;

        int goal_node = -1;
        while (!open.empty() && result.expansions < params.max_expansions)
        {
            const int index = open.top().second;
            open.pop();
            const Node current = nodes[index];
            if (current.g > best[stateKey(current.pose, params)] + 1e-9)
            {
                continue;
            }
            ++result.expansions;

            const double to_goal = std::hypot(current.pose.x - goal.x, current.pose.y - goal.y);
            const bool yaw_ok = !params.use_goal_yaw || std::abs(normalize(current.pose.yaw - goal.yaw)) <= params.goal_yaw_tolerance;
            if (to_goal <= params.goal_tolerance && yaw_ok)
            {
                goal_node = index;
                break;
            }

            if (params.use_goal_yaw && to_goal <= params.shot_distance)
            {
                const double radius = params.wheelbase / std::tan(params.max_steering_angle * params.steering_margin);
                if (const auto dubins = shortestDubinsPath(current.pose, goal, radius))
                {
                    const auto shot = sampleDubinsPath(*dubins, current.pose, params.output_spacing);
                    bool free = true;
                    for (const auto &pose : shot)
                    {
                        if (footprintCollides(grid, pose, params))
                        {
                            free = false;
                            break;
                        }
                    }
                    if (free)
                    {
                        std::vector<PlannerPose> prefix;
                        for (int i = index; i >= 0; i = nodes[i].parent)
                        {
                            prefix.push_back(nodes[i].pose);
                        }
                        std::reverse(prefix.begin(), prefix.end());

                        result.path.push_back(prefix.front());
                        for (size_t i = 1; i < prefix.size(); ++i)
                        {
                            const PlannerPose &a = prefix[i - 1];
                            const PlannerPose &b = prefix[i];
                            const int pieces = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / params.output_spacing)));
                            for (int k = 1; k <= pieces; ++k)
                            {
                                const double t = static_cast<double>(k) / pieces;
                                result.path.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, normalize(a.yaw + normalize(b.yaw - a.yaw) * t)});
                            }
                        }
                        result.path.insert(result.path.end(), shot.begin(), shot.end());
                        result.success = true;
                        return result;
                    }
                }
            }

            for (double kappa : curvatures)
            {
                PlannerPose next = current.pose;
                const double yaw_mid = current.pose.yaw + 0.5 * kappa * params.step;
                next.x += params.step * std::cos(yaw_mid);
                next.y += params.step * std::sin(yaw_mid);
                next.yaw = normalize(current.pose.yaw + kappa * params.step);

                const bool near_start = std::hypot(next.x - start.x, next.y - start.y) < params.start_clearance_radius;
                PlannerPose midpoint = current.pose;
                midpoint.x += 0.5 * params.step * std::cos(current.pose.yaw + 0.25 * kappa * params.step);
                midpoint.y += 0.5 * params.step * std::sin(current.pose.yaw + 0.25 * kappa * params.step);
                midpoint.yaw = normalize(current.pose.yaw + 0.5 * kappa * params.step);
                if (!near_start && (footprintCollides(grid, next, params) || footprintCollides(grid, midpoint, params)))
                {
                    continue;
                }

                const int cell_cost = std::max(grid.cost(next.x, next.y), 0);
                const double step_cost = params.step * (1.0 + params.cost_weight * cell_cost / 100.0) + params.steer_penalty * std::abs(kappa) * params.step +
                                         params.steer_change_penalty * std::abs(kappa - current.curvature) * params.step;
                const double g = current.g + step_cost;
                const long long key = stateKey(next, params);
                const auto found = best.find(key);
                if (found != best.end() && found->second <= g)
                {
                    continue;
                }
                const double h = heuristic(next);
                if (!std::isfinite(h))
                {
                    continue;
                }
                best[key] = g;
                nodes.push_back({next, g, index, kappa});
                open.emplace(g + params.heuristic_weight * h, static_cast<int>(nodes.size()) - 1);
            }
        }

        if (goal_node < 0)
        {
            return result;
        }

        std::vector<PlannerPose> waypoints;
        for (int i = goal_node; i >= 0; i = nodes[i].parent)
        {
            waypoints.push_back(nodes[i].pose);
        }
        std::reverse(waypoints.begin(), waypoints.end());

        result.path.push_back(waypoints.front());
        for (size_t i = 1; i < waypoints.size(); ++i)
        {
            const PlannerPose &a = waypoints[i - 1];
            const PlannerPose &b = waypoints[i];
            const double length = std::hypot(b.x - a.x, b.y - a.y);
            const int pieces = std::max(1, static_cast<int>(std::ceil(length / params.output_spacing)));
            for (int k = 1; k <= pieces; ++k)
            {
                const double t = static_cast<double>(k) / pieces;
                result.path.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, normalize(a.yaw + normalize(b.yaw - a.yaw) * t)});
            }
        }
        result.success = true;
        return result;
    }
}
