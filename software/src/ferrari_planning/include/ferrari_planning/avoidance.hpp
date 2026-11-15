#pragma once

#include <vector>

#include "ferrari_planning/cost_grid.hpp"
#include "ferrari_planning/hybrid_astar.hpp"

namespace ferrari_planning
{
    struct AvoidanceParams
    {
        double max_offset = 0.6;
        double offset_step = 0.1;
        double ramp_length = 2.0;
        double lethal_threshold = 99.0;
        double cost_weight = 1.0;
        double deviation_weight = 0.3;
        double change_weight = 2.0;
        double start_offset_weight = 0.5;
        std::vector<double> footprint_offsets{0.0, 0.15, 0.30};
    };

    struct AvoidanceResult
    {
        bool feasible = false;
        double offset = 0.0;
        double cost = 0.0;
        std::vector<PlannerPose> path;
    };

    int maxPathCost(const std::vector<PlannerPose> &path, const CostGrid &grid, const AvoidanceParams &params);

    size_t firstCollisionIndex(const std::vector<PlannerPose> &path, const CostGrid &grid, const AvoidanceParams &params);

    AvoidanceResult planAvoidance(const std::vector<PlannerPose> &base, const CostGrid &grid, const AvoidanceParams &params, double start_offset, double previous_offset);
}
