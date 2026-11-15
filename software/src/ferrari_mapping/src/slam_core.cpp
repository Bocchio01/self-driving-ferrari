#include "ferrari_mapping/slam_core.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

namespace ferrari_mapping
{
    SlamCore::SlamCore(const SlamParams &params) : params_(params), grid_(params.grid), matcher_(params.matcher) {}

    void SlamCore::setGrid(const OccupancyGrid &grid)
    {
        grid_ = grid;
        field_ = LikelihoodField();
        has_keyframe_ = false;
    }

    void SlamCore::integrate(const std::vector<Eigen::Vector2d> &points_base, const Eigen::Vector2d &sensor_origin_base, const Pose2D &pose_map)
    {
        const double c = std::cos(pose_map.yaw), s = std::sin(pose_map.yaw);
        auto toWorld = [&](const Eigen::Vector2d &p)
        {
            return Eigen::Vector2d(pose_map.x + c * p.x() - s * p.y(), pose_map.y + s * p.x() + c * p.y());
        };

        std::vector<Eigen::Vector2d> hits;
        hits.reserve(points_base.size());
        for (const auto &p : points_base)
        {
            hits.push_back(toWorld(p));
        }
        grid_.integrate(toWorld(sensor_origin_base), hits);
        field_.update(grid_, pose_map.x, pose_map.y, params_.likelihood_radius, params_.likelihood_sigma);
        last_keyframe_ = pose_map;
        has_keyframe_ = true;
        ++keyframes_;
    }

    SlamResult SlamCore::process(const std::vector<Eigen::Vector2d> &points_base, const Eigen::Vector2d &sensor_origin_base, const Pose2D &pose_odom)
    {
        SlamResult result;
        ++scans_;

        Pose2D pose_map = compose(map_to_odom_, pose_odom);
        const bool has_map = !field_.empty();

        if (has_map)
        {
            const MatchResult match = matcher_.match(field_, points_base, pose_map);
            result.score = match.score;
            result.matched = match.ok;
            if (match.ok)
            {
                pose_map = match.pose;
                map_to_odom_ = compose(pose_map, inverse(pose_odom));
            }
        }

        bool integrate_now = false;
        if (params_.mapping)
        {
            const bool bootstrap = scans_ <= params_.bootstrap_scans && !has_map;
            if (!has_keyframe_ || bootstrap)
            {
                integrate_now = true;
            }
            else if (result.matched)
            {
                const double distance = std::hypot(pose_map.x - last_keyframe_.x, pose_map.y - last_keyframe_.y);
                const double angle = std::abs(normalizeAngle(pose_map.yaw - last_keyframe_.yaw));
                integrate_now = distance >= params_.keyframe_distance || angle >= params_.keyframe_angle;
            }
        }
        else if (!has_map && !grid_.logOdds().empty())
        {
            field_.update(grid_, pose_map.x, pose_map.y, params_.likelihood_radius, params_.likelihood_sigma);
        }

        if (integrate_now)
        {
            integrate(points_base, sensor_origin_base, pose_map);
            result.keyframe = true;
        }
        else if (!params_.mapping && has_map)
        {
            const double distance = has_keyframe_ ? std::hypot(pose_map.x - last_keyframe_.x, pose_map.y - last_keyframe_.y) : 1e9;
            if (distance >= 2.0)
            {
                field_.update(grid_, pose_map.x, pose_map.y, params_.likelihood_radius, params_.likelihood_sigma);
                last_keyframe_ = pose_map;
                has_keyframe_ = true;
            }
        }

        result.pose_map = pose_map;
        result.map_to_odom = map_to_odom_;
        return result;
    }

    bool saveMap(const OccupancyGrid &grid, const std::string &prefix)
    {
        const auto &p = grid.params();
        cv::Mat image(grid.height(), grid.width(), CV_8U);
        for (int y = 0; y < grid.height(); ++y)
        {
            for (int x = 0; x < grid.width(); ++x)
            {
                const float l = grid.logOdds().at<float>(y, x);
                uint8_t value = 205;
                if (l > p.occupied_threshold)
                {
                    value = 0;
                }
                else if (l < -0.2f)
                {
                    value = 254;
                }
                image.at<uint8_t>(grid.height() - 1 - y, x) = value;
            }
        }
        if (!cv::imwrite(prefix + ".pgm", image))
        {
            return false;
        }

        std::ofstream yaml(prefix + ".yaml");
        if (!yaml)
        {
            return false;
        }
        const std::string name = prefix.substr(prefix.find_last_of('/') + 1);
        yaml << "image: " << name << ".pgm\nmode: trinary\nresolution: " << p.resolution << "\norigin: [" << p.origin_x << ", " << p.origin_y
             << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
        return true;
    }

    bool loadMap(const std::string &yaml_path, OccupancyGrid &grid)
    {
        std::ifstream yaml(yaml_path);
        if (!yaml)
        {
            return false;
        }
        std::string line, image_name;
        double resolution = 0.05, origin_x = 0.0, origin_y = 0.0;
        while (std::getline(yaml, line))
        {
            const auto colon = line.find(':');
            if (colon == std::string::npos)
            {
                continue;
            }
            const std::string key = line.substr(0, colon);
            std::string value = line.substr(colon + 1);
            value.erase(0, value.find_first_not_of(' '));
            if (key == "image")
            {
                image_name = value;
            }
            else if (key == "resolution")
            {
                resolution = std::stod(value);
            }
            else if (key == "origin")
            {
                std::string cleaned = value;
                for (char &ch : cleaned)
                {
                    if (ch == '[' || ch == ']' || ch == ',')
                    {
                        ch = ' ';
                    }
                }
                std::istringstream stream(cleaned);
                stream >> origin_x >> origin_y;
            }
        }

        const std::string directory = yaml_path.substr(0, yaml_path.find_last_of('/') + 1);
        const cv::Mat image = cv::imread(image_name.front() == '/' ? image_name : directory + image_name, cv::IMREAD_GRAYSCALE);
        if (image.empty())
        {
            return false;
        }

        GridParams params = grid.params();
        params.resolution = resolution;
        params.origin_x = origin_x;
        params.origin_y = origin_y;
        params.size_x_m = image.cols * resolution;
        params.size_y_m = image.rows * resolution;
        OccupancyGrid loaded(params);
        for (int y = 0; y < loaded.height(); ++y)
        {
            for (int x = 0; x < loaded.width(); ++x)
            {
                const uint8_t value = image.at<uint8_t>(image.rows - 1 - y, x);
                float &cell = loaded.logOdds().at<float>(y, x);
                cell = value == 0 ? params.log_odds_max : (value >= 250 ? params.log_odds_min : 0.0f);
            }
        }
        grid = loaded;
        return true;
    }
}
