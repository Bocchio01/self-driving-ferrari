#include "ferrari_mapping/costmap.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace ferrari_mapping
{
    cv::Mat inflateObstacles(const cv::Mat &obstacle_mask, double resolution, const InflationParams &params)
    {
        cv::Mat free_mask = obstacle_mask == 0;
        cv::Mat distance;
        cv::distanceTransform(free_mask, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        distance *= static_cast<float>(resolution);

        cv::Mat cost(obstacle_mask.size(), CV_8U, cv::Scalar(0));
        for (int y = 0; y < cost.rows; ++y)
        {
            const float *d = distance.ptr<float>(y);
            uint8_t *c = cost.ptr<uint8_t>(y);
            for (int x = 0; x < cost.cols; ++x)
            {
                if (d[x] <= params.inscribed_radius + 1e-3)
                {
                    c[x] = kLethalCost;
                }
                else if (d[x] < params.inflation_radius)
                {
                    c[x] = static_cast<uint8_t>(std::lround(99.0 * std::exp(-params.decay * (d[x] - params.inscribed_radius))));
                }
            }
        }
        return cost;
    }

    RollingCostmap::RollingCostmap(double size_m, double resolution, const InflationParams &inflation, double memory_s)
        : size_m_(size_m), resolution_(resolution), inflation_(inflation), memory_s_(memory_s) {}

    void RollingCostmap::addScan(double stamp_s, const std::vector<Eigen::Vector2d> &hits)
    {
        scans_.push_back({stamp_s, hits});
    }

    CostmapData RollingCostmap::build(double center_x, double center_y, double now_s)
    {
        while (!scans_.empty() && now_s - scans_.front().stamp > memory_s_)
        {
            scans_.pop_front();
        }

        CostmapData data;
        data.resolution = resolution_;
        const int cells = static_cast<int>(std::ceil(size_m_ / resolution_));
        data.origin_x = std::floor((center_x - size_m_ / 2.0) / resolution_) * resolution_;
        data.origin_y = std::floor((center_y - size_m_ / 2.0) / resolution_) * resolution_;

        cv::Mat mask = cv::Mat::zeros(cells, cells, CV_8U);
        for (const auto &scan : scans_)
        {
            for (const auto &hit : scan.hits)
            {
                const int ix = static_cast<int>(std::floor((hit.x() - data.origin_x) / resolution_));
                const int iy = static_cast<int>(std::floor((hit.y() - data.origin_y) / resolution_));
                if (ix >= 0 && iy >= 0 && ix < cells && iy < cells)
                {
                    mask.at<uint8_t>(iy, ix) = 255;
                }
            }
        }
        data.cost = inflateObstacles(mask, resolution_, inflation_);
        return data;
    }
}
