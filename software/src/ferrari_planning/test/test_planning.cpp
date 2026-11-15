#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "ferrari_planning/avoidance.hpp"
#include "ferrari_planning/hybrid_astar.hpp"

using namespace ferrari_planning;

namespace
{
    struct Box
    {
        double x0, y0, x1, y1;
    };

    CostGrid makeGrid(double size_x, double size_y, const std::vector<Box> &boxes, double origin_x = 0.0, double origin_y = 0.0, double inflate = 0.15)
    {
        CostGrid grid;
        grid.resolution = 0.05;
        grid.origin_x = origin_x;
        grid.origin_y = origin_y;
        grid.width = static_cast<int>(size_x / grid.resolution);
        grid.height = static_cast<int>(size_y / grid.resolution);
        grid.data.assign(static_cast<size_t>(grid.width) * grid.height, 0);
        for (int iy = 0; iy < grid.height; ++iy)
        {
            for (int ix = 0; ix < grid.width; ++ix)
            {
                const double x = origin_x + (ix + 0.5) * grid.resolution, y = origin_y + (iy + 0.5) * grid.resolution;
                for (const auto &b : boxes)
                {
                    const double dx = std::max({b.x0 - x, 0.0, x - b.x1});
                    const double dy = std::max({b.y0 - y, 0.0, y - b.y1});
                    const double d = std::hypot(dx, dy);
                    if (d <= inflate)
                    {
                        grid.data[static_cast<size_t>(iy) * grid.width + ix] = 100;
                    }
                    else if (d <= inflate + 0.25)
                    {
                        const int value = static_cast<int>(60.0 * (1.0 - (d - inflate) / 0.25));
                        auto &cell = grid.data[static_cast<size_t>(iy) * grid.width + ix];
                        cell = static_cast<int8_t>(std::max<int>(cell, value));
                    }
                }
            }
        }
        return grid;
    }

    double maxCurvature(const std::vector<PlannerPose> &path)
    {
        double worst = 0.0;
        for (size_t i = 2; i < path.size(); ++i)
        {
            const double ds = std::hypot(path[i].x - path[i - 2].x, path[i].y - path[i - 2].y);
            const double dyaw = std::abs(std::remainder(path[i].yaw - path[i - 2].yaw, 2.0 * M_PI));
            if (ds > 1e-6)
            {
                worst = std::max(worst, dyaw / ds);
            }
        }
        return worst;
    }

    bool collides(const CostGrid &grid, const std::vector<PlannerPose> &path, double lethal = 99.0)
    {
        for (const auto &p : path)
        {
            for (double offset : {0.0, 0.15, 0.30})
            {
                const int cost = grid.cost(p.x + std::cos(p.yaw) * offset, p.y + std::sin(p.yaw) * offset);
                if (cost < 0 || cost >= lethal)
                {
                    return true;
                }
            }
        }
        return false;
    }
}

TEST(HybridAStar, ReachesAGoalInFreeSpaceWithBoundedCurvature)
{
    const CostGrid grid = makeGrid(10.0, 10.0, {});
    const PlanResult result = planHybridAStar(grid, {1.0, 1.0, 0.0}, {8.0, 4.0, 0.0}, HybridAStarParams());

    ASSERT_TRUE(result.success);
    EXPECT_NEAR(result.path.front().x, 1.0, 1e-9);
    EXPECT_LT(std::hypot(result.path.back().x - 8.0, result.path.back().y - 4.0), 0.25);
    EXPECT_LT(maxCurvature(result.path), 1.0 / 0.6 + 0.4);
    EXPECT_FALSE(collides(grid, result.path));
}

TEST(HybridAStar, RoutesThroughTheGapOfAWall)
{
    const CostGrid grid = makeGrid(12.0, 10.0, {{5.8, 0.0, 6.2, 6.5}, {5.8, 8.5, 6.2, 10.0}});
    const PlanResult result = planHybridAStar(grid, {1.0, 2.0, 0.0}, {11.0, 2.0, 0.0}, HybridAStarParams());

    ASSERT_TRUE(result.success);
    bool crossed_in_gap = false;
    for (const auto &p : result.path)
    {
        if (std::abs(p.x - 6.0) < 0.1)
        {
            crossed_in_gap = p.y > 6.6 && p.y < 8.4;
        }
    }
    EXPECT_TRUE(crossed_in_gap);
    EXPECT_FALSE(collides(grid, result.path));
}

TEST(HybridAStar, TurnsAroundWhenTheGoalIsBehindTheVehicle)
{
    const CostGrid grid = makeGrid(12.0, 12.0, {});
    const PlanResult result = planHybridAStar(grid, {6.0, 6.0, 0.0}, {3.0, 6.0, 0.0}, HybridAStarParams());

    ASSERT_TRUE(result.success);
    EXPECT_GT(result.path.size() * 0.05, 3.5);
    EXPECT_LT(maxCurvature(result.path), 1.0 / 0.6 + 0.4);
}

TEST(HybridAStar, FailsForGoalsInsideObstaclesAndEnclosedGoals)
{
    const CostGrid blocked = makeGrid(10.0, 10.0, {{7.0, 3.0, 9.0, 5.0}});
    EXPECT_FALSE(planHybridAStar(blocked, {1.0, 1.0, 0.0}, {8.0, 4.0, 0.0}, HybridAStarParams()).success);

    const CostGrid enclosed = makeGrid(10.0, 10.0, {{6.0, 3.0, 6.3, 7.0}, {8.7, 3.0, 9.0, 7.0}, {6.0, 3.0, 9.0, 3.3}, {6.0, 6.7, 9.0, 7.0}});
    EXPECT_FALSE(planHybridAStar(enclosed, {1.0, 1.0, 0.0}, {7.5, 5.0, 0.0}, HybridAStarParams()).success);
}

namespace
{
    std::vector<PlannerPose> straightBase(double length, double spacing = 0.05)
    {
        std::vector<PlannerPose> base;
        for (double x = 0.0; x <= length + 1e-9; x += spacing)
        {
            base.push_back({x, 0.0, 0.0});
        }
        return base;
    }

    CostGrid centeredGrid(const std::vector<Box> &boxes)
    {
        return makeGrid(12.0, 6.0, boxes, -1.0, -3.0, 0.12);
    }
}

TEST(Avoidance, KeepsTheBasePathWhenFree)
{
    const auto result = planAvoidance(straightBase(5.0), centeredGrid({}), AvoidanceParams(), 0.0, 0.0);

    ASSERT_TRUE(result.feasible);
    EXPECT_DOUBLE_EQ(result.offset, 0.0);
    EXPECT_NEAR(result.path[40].y, 0.0, 1e-9);
}

TEST(Avoidance, SwervesAroundAnObstacleOnThePath)
{
    const CostGrid grid = centeredGrid({{2.4, -0.15, 2.8, 0.15}});
    const auto result = planAvoidance(straightBase(5.0), grid, AvoidanceParams(), 0.0, 0.0);

    ASSERT_TRUE(result.feasible);
    EXPECT_GE(std::abs(result.offset), 0.3);
    EXPECT_FALSE(collides(grid, result.path));
    EXPECT_NEAR(result.path.front().y, 0.0, 1e-6);
    EXPECT_NEAR(result.path.back().y, 0.0, 1e-6);
    EXPECT_LT(maxCurvature(result.path), 1.0 / 0.6);
}

TEST(Avoidance, KeepsTheSideChosenBefore)
{
    const CostGrid grid = centeredGrid({{2.4, -0.15, 2.8, 0.15}});
    AvoidanceParams params;
    const auto left = planAvoidance(straightBase(5.0), grid, params, 0.0, 0.4);
    const auto right = planAvoidance(straightBase(5.0), grid, params, 0.0, -0.4);

    ASSERT_TRUE(left.feasible);
    ASSERT_TRUE(right.feasible);
    EXPECT_GT(left.offset, 0.0);
    EXPECT_LT(right.offset, 0.0);
}

TEST(Avoidance, ReportsInfeasibleWhenTheCorridorIsBlocked)
{
    const CostGrid grid = centeredGrid({{2.4, -2.9, 2.8, 2.9}});
    const auto result = planAvoidance(straightBase(5.0), grid, AvoidanceParams(), 0.0, 0.0);

    EXPECT_FALSE(result.feasible);
    EXPECT_TRUE(result.path.empty());
}

TEST(Avoidance, BlendsFromTheCurrentLateralOffset)
{
    const auto result = planAvoidance(straightBase(5.0), centeredGrid({}), AvoidanceParams(), 0.3, 0.0);

    ASSERT_TRUE(result.feasible);
    EXPECT_NEAR(result.path.front().y, 0.3, 1e-6);
    EXPECT_NEAR(result.path[60].y, 0.0, 0.02);
}

TEST(Avoidance, FindsTheFirstCollidingPoseOnAPath)
{
    const CostGrid grid = centeredGrid({{2.4, -0.15, 2.8, 0.15}});
    const auto base = straightBase(5.0);

    const size_t index = firstCollisionIndex(base, grid, AvoidanceParams());

    ASSERT_LT(index, base.size());
    EXPECT_NEAR(base[index].x, 2.4 - 0.12 - 0.30, 0.1);
    EXPECT_EQ(firstCollisionIndex(base, centeredGrid({}), AvoidanceParams()), base.size());
}
