#include "ferrari_mapping/scan_matcher.hpp"

#include <cmath>

namespace ferrari_mapping
{
    ScanMatcher::ScanMatcher(const MatcherParams &params) : params_(params) {}

    double ScanMatcher::dataScore(const LikelihoodField &field, const std::vector<Eigen::Vector2d> &points, const Pose2D &pose, bool smooth) const
    {
        const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
        double sum = 0.0;
        for (const auto &p : points)
        {
            const double wx = pose.x + c * p.x() - s * p.y();
            const double wy = pose.y + s * p.x() + c * p.y();
            sum += smooth ? field.bilinear(wx, wy) : field.nearest(wx, wy);
        }
        return points.empty() ? 0.0 : sum / static_cast<double>(points.size());
    }

    double ScanMatcher::objective(const LikelihoodField &field, const std::vector<Eigen::Vector2d> &points, const Pose2D &pose, const Pose2D &prediction, bool smooth) const
    {
        const double dx = pose.x - prediction.x, dy = pose.y - prediction.y;
        const double dyaw = normalizeAngle(pose.yaw - prediction.yaw);
        const double penalty = 0.5 * params_.prior_weight * ((dx * dx + dy * dy) / (params_.prior_sigma_xy * params_.prior_sigma_xy) + dyaw * dyaw / (params_.prior_sigma_yaw * params_.prior_sigma_yaw));
        return dataScore(field, points, pose, smooth) - penalty;
    }

    MatchResult ScanMatcher::match(const LikelihoodField &field, const std::vector<Eigen::Vector2d> &points_base, const Pose2D &prediction) const
    {
        MatchResult result;
        result.pose = prediction;
        if (field.empty() || points_base.empty())
        {
            return result;
        }

        std::vector<Eigen::Vector2d> coarse_points;
        const size_t stride = std::max<size_t>(1, points_base.size() / static_cast<size_t>(params_.coarse_max_points));
        for (size_t i = 0; i < points_base.size(); i += stride)
        {
            coarse_points.push_back(points_base[i]);
        }

        auto search = [&](const std::vector<Eigen::Vector2d> &points, const Pose2D &center, double linear, double angular, double step, double angle_step, bool smooth, Pose2D &best_pose, double &best_value)
        {
            const int nxy = static_cast<int>(std::round(linear / step));
            const int nth = static_cast<int>(std::round(angular / angle_step));
            for (int it = -nth; it <= nth; ++it)
            {
                for (int ix = -nxy; ix <= nxy; ++ix)
                {
                    for (int iy = -nxy; iy <= nxy; ++iy)
                    {
                        const Pose2D candidate{center.x + ix * step, center.y + iy * step, normalizeAngle(center.yaw + it * angle_step)};
                        const double value = objective(field, points, candidate, prediction, smooth);
                        if (value > best_value)
                        {
                            best_value = value;
                            best_pose = candidate;
                        }
                    }
                }
            }
        };

        Pose2D best = prediction;
        double best_value = -1e9;
        search(coarse_points, prediction, params_.linear_window, params_.angular_window, params_.coarse_step, params_.coarse_angle_step, false, best, best_value);

        std::vector<Eigen::Vector2d> fine_points;
        const size_t fine_stride = std::max<size_t>(1, points_base.size() / static_cast<size_t>(params_.fine_max_points));
        for (size_t i = 0; i < points_base.size(); i += fine_stride)
        {
            fine_points.push_back(points_base[i]);
        }

        Pose2D refined = best;
        double refined_value = -1e9;
        search(fine_points, best, params_.coarse_step, params_.coarse_angle_step, params_.fine_step, params_.fine_angle_step, true, refined, refined_value);

        result.pose = refined;
        result.score = dataScore(field, points_base, refined, true);
        result.ok = result.score >= params_.min_score;
        return result;
    }
}
