#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "ferrari_mapping/slam_core.hpp"

using namespace ferrari_mapping;

namespace
{
    struct Segment
    {
        double x1, y1, x2, y2;
    };

    void addRectangle(std::vector<Segment> &segments, double cx, double cy, double w, double h)
    {
        const double x0 = cx - w / 2, x1 = cx + w / 2, y0 = cy - h / 2, y1 = cy + h / 2;
        segments.push_back({x0, y0, x1, y0});
        segments.push_back({x1, y0, x1, y1});
        segments.push_back({x1, y1, x0, y1});
        segments.push_back({x0, y1, x0, y0});
    }

    std::vector<Segment> hallWorld()
    {
        std::vector<Segment> segments;
        addRectangle(segments, 1.75, -4.5, 15.5, 21.0);
        for (const auto &b : std::vector<std::array<double, 4>>{{-4.5, -3.0, 1.0, 1.0}, {8.0, -8.0, 1.2, 1.2}, {1.0, -13.5, 1.5, 1.0}, {7.0, 3.5, 1.0, 1.0}, {-4.0, -10.0, 1.0, 1.6}})
        {
            addRectangle(segments, b[0], b[1], b[2], b[3]);
        }
        return segments;
    }

    double raycast(const std::vector<Segment> &segments, double ox, double oy, double angle)
    {
        const double dx = std::cos(angle), dy = std::sin(angle);
        double best = std::numeric_limits<double>::infinity();
        for (const auto &s : segments)
        {
            const double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
            const double denominator = dx * ey - dy * ex;
            if (std::abs(denominator) < 1e-12)
            {
                continue;
            }
            const double t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / denominator;
            const double u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / denominator;
            if (t > 0 && u >= 0 && u <= 1 && t < best)
            {
                best = t;
            }
        }
        return best;
    }

    const Eigen::Vector2d kLidarInBase(0.11, 0.0);

    std::vector<Eigen::Vector2d> simulateScan(const std::vector<Segment> &world, const Pose2D &base_pose, std::mt19937 &rng, double noise_std = 0.01)
    {
        std::normal_distribution<double> noise(0.0, 1.0);
        const double c = std::cos(base_pose.yaw), s = std::sin(base_pose.yaw);
        const double ox = base_pose.x + c * kLidarInBase.x(), oy = base_pose.y + s * kLidarInBase.x();

        std::vector<Eigen::Vector2d> points;
        const int bins = 450;
        for (int i = 0; i < bins; ++i)
        {
            const double beam = -M_PI + (i + 0.5) * 2.0 * M_PI / bins;
            double range = raycast(world, ox, oy, base_pose.yaw + beam);
            if (!std::isfinite(range) || range > 12.0 || range < 0.02)
            {
                continue;
            }
            range += noise_std * noise(rng);
            points.emplace_back(kLidarInBase.x() + range * std::cos(beam), kLidarInBase.y() + range * std::sin(beam));
        }
        return points;
    }
}

TEST(OccupancyGrid, RaycastMarksFreeAndOccupiedCells)
{
    OccupancyGrid grid;
    grid.integrate(Eigen::Vector2d(0.0, 0.0), {Eigen::Vector2d(2.0, 0.0)});

    int ix, iy;
    grid.worldToCell(1.0, 0.0, ix, iy);
    EXPECT_LT(grid.logOdds().at<float>(iy, ix), 0.0f);
    grid.worldToCell(2.0, 0.0, ix, iy);
    EXPECT_GT(grid.logOdds().at<float>(iy, ix), 0.0f);
    EXPECT_TRUE(grid.isOccupied(ix, iy));
    grid.worldToCell(0.0, 1.0, ix, iy);
    EXPECT_EQ(grid.logOdds().at<float>(iy, ix), 0.0f);
}

TEST(OccupancyGrid, IgnoresHitsOutsideTheMap)
{
    OccupancyGrid grid;
    grid.integrate(Eigen::Vector2d(0.0, 0.0), {Eigen::Vector2d(100.0, 0.0)});
    EXPECT_EQ(cv::countNonZero(grid.logOdds()), 0);
}

TEST(ScanMatcher, RecoversASmallKnownDisplacement)
{
    const auto world = hallWorld();
    std::mt19937 rng(3);
    const Pose2D reference{1.0, -4.0, 0.4};

    std::vector<Eigen::Vector2d> hits;
    for (const auto &p : simulateScan(world, reference, rng))
    {
        const Pose2D w = compose(reference, Pose2D{p.x(), p.y(), 0.0});
        hits.emplace_back(w.x, w.y);
    }
    OccupancyGrid grid;
    grid.integrate(Eigen::Vector2d(reference.x + 0.11 * std::cos(reference.yaw), reference.y + 0.11 * std::sin(reference.yaw)), hits);
    LikelihoodField field;
    field.update(grid, reference.x, reference.y, 14.0, 0.10);

    const Pose2D truth{reference.x + 0.06, reference.y - 0.04, reference.yaw + 0.03};
    const auto scan = simulateScan(world, truth, rng);

    ScanMatcher matcher;
    const MatchResult result = matcher.match(field, scan, reference);

    EXPECT_TRUE(result.ok);
    EXPECT_NEAR(result.pose.x, truth.x, 0.015);
    EXPECT_NEAR(result.pose.y, truth.y, 0.015);
    EXPECT_NEAR(result.pose.yaw, truth.yaw, 0.007);
}

namespace
{
    struct LoopResult
    {
        double max_error = 0.0;
        double final_error = 0.0;
        double odom_final_error = 0.0;
        size_t scans = 0;
        size_t matched = 0;
        double process_ms = 0.0;
    };

    LoopResult driveLoop(double speed_scale_error, double yaw_rate_bias)
    {
        const auto world = hallWorld();
        std::mt19937 rng(11);
        std::normal_distribution<double> normal(0.0, 1.0);

        const double cx = 1.8, cy = -4.5, ax = 4.0, ay = 5.5;
        const double speed = 0.8, dt = 0.1;
        const double theta_rate = speed / std::sqrt((ax * ax + ay * ay) / 2.0);

        auto truthAt = [&](double theta)
        {
            const double x = cx + ax * std::cos(theta), y = cy + ay * std::sin(theta);
            const double dx = -ax * std::sin(theta), dy = ay * std::cos(theta);
            return Pose2D{x, y, std::atan2(dy, dx)};
        };

        SlamParams params;
        SlamCore slam(params);

        const Pose2D start = truthAt(0.0);
        slam.setMapToOdom(start);

        Pose2D odom;
        Pose2D previous = start;
        LoopResult result;
        const int steps = static_cast<int>((2.0 * M_PI / theta_rate) / dt);
        for (int k = 0; k <= steps; ++k)
        {
            const Pose2D truth = truthAt(k * dt * theta_rate);
            if (k > 0)
            {
                const double distance = std::hypot(truth.x - previous.x, truth.y - previous.y);
                const double dyaw = normalizeAngle(truth.yaw - previous.yaw);
                const double measured_distance = distance * (1.0 + speed_scale_error) + 0.002 * normal(rng);
                const double measured_dyaw = dyaw + yaw_rate_bias * dt + 0.002 * normal(rng);
                const double mid = odom.yaw + 0.5 * measured_dyaw;
                odom.x += measured_distance * std::cos(mid);
                odom.y += measured_distance * std::sin(mid);
                odom.yaw = normalizeAngle(odom.yaw + measured_dyaw);
            }
            previous = truth;

            const auto scan = simulateScan(world, truth, rng);
            const auto t0 = std::chrono::steady_clock::now();
            const SlamResult step = slam.process(scan, kLidarInBase, odom);
            result.process_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

            const double error = std::hypot(step.pose_map.x - truth.x, step.pose_map.y - truth.y);
            result.max_error = std::max(result.max_error, error);
            result.final_error = error;
            const Pose2D odom_only = compose(start, odom);
            result.odom_final_error = std::hypot(odom_only.x - truth.x, odom_only.y - truth.y);
            ++result.scans;
            result.matched += step.matched ? 1 : 0;
        }
        return result;
    }
}

TEST(SlamCore, StaysLockedToTheWorldWhileOdometryDrifts)
{
    const LoopResult result = driveLoop(0.03, 0.03);

    EXPECT_GT(result.odom_final_error, 1.5);
    EXPECT_LT(result.max_error, 0.10);
    EXPECT_LT(result.final_error, 0.06);
    EXPECT_GT(static_cast<double>(result.matched) / result.scans, 0.95);
    std::cout << "mean process time ms: " << result.process_ms / result.scans << " scans " << result.scans << " max err " << result.max_error << " final err " << result.final_error << " odom err " << result.odom_final_error << std::endl;
}

TEST(SlamCore, ExactOdometryDoesNotDegradeTheTrajectory)
{
    const LoopResult result = driveLoop(0.0, 0.0);
    EXPECT_LT(result.max_error, 0.05);
}

TEST(SlamCore, MapRoundTripsThroughFiles)
{
    OccupancyGrid grid;
    grid.integrate(Eigen::Vector2d(0.0, 0.0), {Eigen::Vector2d(2.0, 0.5), Eigen::Vector2d(-1.0, 1.5)});

    const std::string prefix = "/tmp/ferrari_map_roundtrip_test";
    ASSERT_TRUE(saveMap(grid, prefix));

    OccupancyGrid loaded;
    ASSERT_TRUE(loadMap(prefix + ".yaml", loaded));

    int ix, iy;
    ASSERT_TRUE(loaded.worldToCell(2.0, 0.5, ix, iy));
    EXPECT_TRUE(loaded.isOccupied(ix, iy));
    ASSERT_TRUE(loaded.worldToCell(1.0, 0.25, ix, iy));
    EXPECT_LT(loaded.logOdds().at<float>(iy, ix), 0.0f);
    EXPECT_EQ(loaded.width(), grid.width());
}
