#pragma once

#include <vector>

#include "ferrari_planning/cost_grid.hpp"

namespace ferrari_planning
{
    struct PlannerPose
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct HybridAStarParams
    {
        double wheelbase = 0.218;
        double max_steering_angle = 0.3448;
        double step = 0.25;
        int steering_samples_per_side = 2;
        double lethal_threshold = 99.0;
        double cost_weight = 3.0;
        double steer_penalty = 0.3;
        double steer_change_penalty = 0.5;
        double goal_tolerance = 0.20;
        double goal_yaw_tolerance = 0.6;
        bool use_goal_yaw = false;
        double xy_resolution = 0.10;
        int yaw_bins = 72;
        int max_expansions = 60000;
        double heuristic_weight = 1.2;
        std::vector<double> footprint_offsets{0.0, 0.15, 0.30};
        double output_spacing = 0.05;
        double start_clearance_radius = 0.4;
    };

    struct PlanResult
    {
        bool success = false;
        std::vector<PlannerPose> path;
        int expansions = 0;
    };

    PlanResult planHybridAStar(const CostGrid &grid, const PlannerPose &start, const PlannerPose &goal, const HybridAStarParams &params);
}
