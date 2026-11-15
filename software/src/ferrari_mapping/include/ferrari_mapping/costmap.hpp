#pragma once

#include <deque>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace ferrari_mapping
{
    struct InflationParams
    {
        double inscribed_radius = 0.10;
        double inflation_radius = 0.35;
        double decay = 6.0;
    };

    constexpr uint8_t kLethalCost = 100;

    cv::Mat inflateObstacles(const cv::Mat &obstacle_mask, double resolution, const InflationParams &params);

    struct CostmapData
    {
        cv::Mat cost;
        double origin_x = 0.0;
        double origin_y = 0.0;
        double resolution = 0.05;
    };

    class RollingCostmap
    {
    public:
        RollingCostmap(double size_m, double resolution, const InflationParams &inflation, double memory_s);

        void addScan(double stamp_s, const std::vector<Eigen::Vector2d> &hits);
        CostmapData build(double center_x, double center_y, double now_s);

    private:
        struct TimedScan
        {
            double stamp;
            std::vector<Eigen::Vector2d> hits;
        };

        double size_m_;
        double resolution_;
        InflationParams inflation_;
        double memory_s_;
        std::deque<TimedScan> scans_;
    };
}
