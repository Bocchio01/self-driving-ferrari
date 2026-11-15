#pragma once

#include <vector>

#include <Eigen/Dense>

#include "ferrari_mapping/occupancy_grid.hpp"
#include "ferrari_mapping/pose2d.hpp"

namespace ferrari_mapping
{
    struct MatcherParams
    {
        double linear_window = 0.12;
        double angular_window = 0.07;
        double coarse_step = 0.04;
        double coarse_angle_step = 0.0175;
        double fine_step = 0.01;
        double fine_angle_step = 0.0035;
        int coarse_max_points = 120;
        int fine_max_points = 240;
        double prior_weight = 0.05;
        double prior_sigma_xy = 0.10;
        double prior_sigma_yaw = 0.05;
        double min_score = 0.25;
    };

    struct MatchResult
    {
        Pose2D pose;
        double score = 0.0;
        bool ok = false;
    };

    class ScanMatcher
    {
    public:
        explicit ScanMatcher(const MatcherParams &params = MatcherParams());

        MatchResult match(const LikelihoodField &field, const std::vector<Eigen::Vector2d> &points_base, const Pose2D &prediction) const;

    private:
        double dataScore(const LikelihoodField &field, const std::vector<Eigen::Vector2d> &points, const Pose2D &pose, bool smooth) const;
        double objective(const LikelihoodField &field, const std::vector<Eigen::Vector2d> &points, const Pose2D &pose, const Pose2D &prediction, bool smooth) const;

        MatcherParams params_;
    };
}
