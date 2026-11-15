#pragma once

#include <optional>
#include <vector>

#include "ferrari_planning/hybrid_astar.hpp"

namespace ferrari_planning
{
    struct DubinsPath
    {
        int word = 0;
        double t = 0.0;
        double p = 0.0;
        double q = 0.0;
        double radius = 1.0;

        double length() const { return (t + p + q) * radius; }
    };

    std::optional<DubinsPath> shortestDubinsPath(const PlannerPose &start, const PlannerPose &goal, double radius);
    std::vector<PlannerPose> sampleDubinsPath(const DubinsPath &path, const PlannerPose &start, double spacing);
}
