#include <gtest/gtest.h>

#include "ferrari_mapping/costmap.hpp"

using namespace ferrari_mapping;

TEST(Inflation, CostsFollowDistanceFromObstacle)
{
    cv::Mat mask = cv::Mat::zeros(100, 100, CV_8U);
    mask.at<uint8_t>(50, 50) = 255;
    InflationParams params;
    const cv::Mat cost = inflateObstacles(mask, 0.05, params);

    EXPECT_EQ(cost.at<uint8_t>(50, 50), kLethalCost);
    EXPECT_EQ(cost.at<uint8_t>(50, 51), kLethalCost);
    EXPECT_EQ(cost.at<uint8_t>(50, 52), kLethalCost);
    const int inside = cost.at<uint8_t>(50, 54);
    const int farther = cost.at<uint8_t>(50, 56);
    EXPECT_GT(inside, farther);
    EXPECT_GT(farther, 0);
    EXPECT_LT(inside, kLethalCost);
    EXPECT_EQ(cost.at<uint8_t>(50, 60), 0);
    EXPECT_EQ(cost.at<uint8_t>(10, 10), 0);
}

TEST(Inflation, NoObstaclesMeansNoCost)
{
    const cv::Mat cost = inflateObstacles(cv::Mat::zeros(40, 40, CV_8U), 0.05, InflationParams());
    EXPECT_EQ(cv::countNonZero(cost), 0);
}

TEST(RollingCostmap, KeepsRecentScansAndForgetsOldOnes)
{
    RollingCostmap costmap(6.0, 0.05, InflationParams(), 0.5);
    costmap.addScan(0.0, {Eigen::Vector2d(1.0, 0.0)});
    costmap.addScan(0.4, {Eigen::Vector2d(0.0, 1.0)});

    CostmapData data = costmap.build(0.0, 0.0, 0.45);
    auto at = [&](double x, double y)
    {
        return data.cost.at<uint8_t>(static_cast<int>((y - data.origin_y) / data.resolution), static_cast<int>((x - data.origin_x) / data.resolution));
    };
    EXPECT_EQ(at(1.0, 0.0), kLethalCost);
    EXPECT_EQ(at(0.0, 1.0), kLethalCost);

    data = costmap.build(0.0, 0.0, 0.8);
    EXPECT_EQ(at(1.0, 0.0), 0);
    EXPECT_EQ(at(0.0, 1.0), kLethalCost);
}

TEST(RollingCostmap, WindowFollowsTheCenterOnTheGrid)
{
    RollingCostmap costmap(6.0, 0.05, InflationParams(), 1.0);
    const CostmapData a = costmap.build(0.0, 0.0, 0.0);
    const CostmapData b = costmap.build(0.62, -1.03, 0.0);

    EXPECT_NEAR(std::remainder(b.origin_x - a.origin_x, 0.05), 0.0, 1e-9) << "origin must stay aligned to the grid";
    EXPECT_LT(b.origin_y, a.origin_y);
    EXPECT_EQ(a.cost.rows, 120);
}
