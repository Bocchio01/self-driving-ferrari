#include "ferrari_mapping/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace ferrari_mapping
{
    OccupancyGrid::OccupancyGrid(const GridParams &params) : params_(params)
    {
        const int width = static_cast<int>(std::ceil(params_.size_x_m / params_.resolution));
        const int height = static_cast<int>(std::ceil(params_.size_y_m / params_.resolution));
        log_odds_ = cv::Mat::zeros(height, width, CV_32F);
    }

    bool OccupancyGrid::worldToCell(double x, double y, int &ix, int &iy) const
    {
        ix = static_cast<int>(std::floor((x - params_.origin_x) / params_.resolution));
        iy = static_cast<int>(std::floor((y - params_.origin_y) / params_.resolution));
        return ix >= 0 && iy >= 0 && ix < width() && iy < height();
    }

    Eigen::Vector2d OccupancyGrid::cellToWorld(int ix, int iy) const
    {
        return {params_.origin_x + (ix + 0.5) * params_.resolution, params_.origin_y + (iy + 0.5) * params_.resolution};
    }

    void OccupancyGrid::traceRay(int x0, int y0, int x1, int y1)
    {
        const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int error = dx - dy;
        int x = x0, y = y0;
        while (x != x1 || y != y1)
        {
            float &cell = log_odds_.at<float>(y, x);
            cell = std::max(cell + params_.log_odds_free, params_.log_odds_min);
            const int doubled = 2 * error;
            if (doubled > -dy)
            {
                error -= dy;
                x += sx;
            }
            if (doubled < dx)
            {
                error += dx;
                y += sy;
            }
        }
        float &hit = log_odds_.at<float>(y1, x1);
        hit = std::min(hit + params_.log_odds_hit, params_.log_odds_max);
    }

    void OccupancyGrid::integrate(const Eigen::Vector2d &sensor_origin, const std::vector<Eigen::Vector2d> &hits)
    {
        int ox, oy;
        if (!worldToCell(sensor_origin.x(), sensor_origin.y(), ox, oy))
        {
            return;
        }
        for (const auto &hit : hits)
        {
            int hx, hy;
            if (worldToCell(hit.x(), hit.y(), hx, hy))
            {
                traceRay(ox, oy, hx, hy);
            }
        }
    }

    std::vector<int8_t> OccupancyGrid::toOccupancyData() const
    {
        std::vector<int8_t> data(static_cast<size_t>(width()) * height());
        for (int y = 0; y < height(); ++y)
        {
            for (int x = 0; x < width(); ++x)
            {
                const float l = log_odds_.at<float>(y, x);
                data[static_cast<size_t>(y) * width() + x] = l == 0.0f ? -1 : static_cast<int8_t>(std::lround(100.0 / (1.0 + std::exp(-l))));
            }
        }
        return data;
    }

    void LikelihoodField::update(const OccupancyGrid &grid, double center_x, double center_y, double radius_m, double sigma_m)
    {
        const auto &p = grid.params();
        resolution_ = p.resolution;
        int cx, cy;
        grid.worldToCell(center_x, center_y, cx, cy);
        const int radius = static_cast<int>(std::ceil(radius_m / p.resolution));
        const int x0 = std::max(0, cx - radius), x1 = std::min(grid.width(), cx + radius);
        const int y0 = std::max(0, cy - radius), y1 = std::min(grid.height(), cy + radius);
        if (x1 <= x0 || y1 <= y0)
        {
            field_.release();
            return;
        }

        const cv::Rect window(x0, y0, x1 - x0, y1 - y0);
        cv::Mat free_mask = grid.logOdds()(window) <= p.occupied_threshold;
        cv::Mat free_u8;
        free_mask.convertTo(free_u8, CV_8U, 255);

        cv::Mat distance;
        cv::distanceTransform(free_u8, distance, cv::DIST_L2, cv::DIST_MASK_3);
        distance *= static_cast<float>(p.resolution);

        cv::Mat scaled = distance.mul(distance) * static_cast<float>(-0.5 / (sigma_m * sigma_m));
        cv::exp(scaled, field_);

        origin_x_ = p.origin_x + x0 * p.resolution;
        origin_y_ = p.origin_y + y0 * p.resolution;
    }

    double LikelihoodField::nearest(double x, double y) const
    {
        const int ix = static_cast<int>((x - origin_x_) / resolution_);
        const int iy = static_cast<int>((y - origin_y_) / resolution_);
        if (x < origin_x_ || y < origin_y_ || ix >= field_.cols || iy >= field_.rows)
        {
            return 0.0;
        }
        return field_.at<float>(iy, ix);
    }

    double LikelihoodField::bilinear(double x, double y) const
    {
        const double fx = (x - origin_x_) / resolution_ - 0.5;
        const double fy = (y - origin_y_) / resolution_ - 0.5;
        const int ix = static_cast<int>(std::floor(fx));
        const int iy = static_cast<int>(std::floor(fy));
        if (ix < 0 || iy < 0 || ix + 1 >= field_.cols || iy + 1 >= field_.rows)
        {
            return nearest(x, y);
        }
        const double tx = fx - ix, ty = fy - iy;
        const double a = field_.at<float>(iy, ix), b = field_.at<float>(iy, ix + 1);
        const double c = field_.at<float>(iy + 1, ix), d = field_.at<float>(iy + 1, ix + 1);
        return (1 - ty) * ((1 - tx) * a + tx * b) + ty * ((1 - tx) * c + tx * d);
    }
}
