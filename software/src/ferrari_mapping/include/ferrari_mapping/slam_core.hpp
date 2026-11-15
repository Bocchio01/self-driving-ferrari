#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ferrari_mapping/occupancy_grid.hpp"
#include "ferrari_mapping/pose2d.hpp"
#include "ferrari_mapping/scan_matcher.hpp"

namespace ferrari_mapping
{
    struct SlamParams
    {
        GridParams grid;
        MatcherParams matcher;
        double likelihood_sigma = 0.10;
        double likelihood_radius = 14.0;
        double keyframe_distance = 0.10;
        double keyframe_angle = 0.087;
        int bootstrap_scans = 3;
        bool mapping = true;
    };

    struct SlamResult
    {
        Pose2D pose_map;
        Pose2D map_to_odom;
        double score = 0.0;
        bool matched = false;
        bool keyframe = false;
    };

    class SlamCore
    {
    public:
        explicit SlamCore(const SlamParams &params = SlamParams());

        SlamResult process(const std::vector<Eigen::Vector2d> &points_base, const Eigen::Vector2d &sensor_origin_base, const Pose2D &pose_odom);

        void setMapToOdom(const Pose2D &map_to_odom) { map_to_odom_ = map_to_odom; }
        void setMapping(bool mapping) { params_.mapping = mapping; }
        void setGrid(const OccupancyGrid &grid);
        const OccupancyGrid &grid() const { return grid_; }
        const Pose2D &mapToOdom() const { return map_to_odom_; }
        size_t keyframeCount() const { return keyframes_; }

    private:
        void integrate(const std::vector<Eigen::Vector2d> &points_base, const Eigen::Vector2d &sensor_origin_base, const Pose2D &pose_map);

        SlamParams params_;
        OccupancyGrid grid_;
        LikelihoodField field_;
        ScanMatcher matcher_;
        Pose2D map_to_odom_;
        Pose2D last_keyframe_;
        bool has_keyframe_ = false;
        size_t keyframes_ = 0;
        int scans_ = 0;
    };

    bool saveMap(const OccupancyGrid &grid, const std::string &prefix);
    bool loadMap(const std::string &yaml_path, OccupancyGrid &grid);
}
