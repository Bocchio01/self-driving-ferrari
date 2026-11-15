#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace ferrari_mapping
{
    struct GridParams
    {
        double size_x_m = 50.0;
        double size_y_m = 50.0;
        double resolution = 0.05;
        double origin_x = -25.0;
        double origin_y = -25.0;
        float log_odds_hit = 0.85f;
        float log_odds_free = -0.4f;
        float log_odds_min = -4.0f;
        float log_odds_max = 4.0f;
        float occupied_threshold = 0.6f;
    };

    class OccupancyGrid
    {
    public:
        explicit OccupancyGrid(const GridParams &params = GridParams());

        int width() const { return log_odds_.cols; }
        int height() const { return log_odds_.rows; }
        const GridParams &params() const { return params_; }
        const cv::Mat &logOdds() const { return log_odds_; }
        cv::Mat &logOdds() { return log_odds_; }

        bool worldToCell(double x, double y, int &ix, int &iy) const;
        Eigen::Vector2d cellToWorld(int ix, int iy) const;

        void integrate(const Eigen::Vector2d &sensor_origin, const std::vector<Eigen::Vector2d> &hits);

        bool isOccupied(int ix, int iy) const { return log_odds_.at<float>(iy, ix) > params_.occupied_threshold; }
        std::vector<int8_t> toOccupancyData() const;

    private:
        void traceRay(int x0, int y0, int x1, int y1);

        GridParams params_;
        cv::Mat log_odds_;
    };

    class LikelihoodField
    {
    public:
        void update(const OccupancyGrid &grid, double center_x, double center_y, double radius_m, double sigma_m);

        double nearest(double x, double y) const;
        double bilinear(double x, double y) const;
        bool empty() const { return field_.empty(); }

    private:
        cv::Mat field_;
        double origin_x_ = 0.0;
        double origin_y_ = 0.0;
        double resolution_ = 0.05;
    };
}
