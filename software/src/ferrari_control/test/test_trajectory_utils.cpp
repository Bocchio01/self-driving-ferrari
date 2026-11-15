#include <cmath>

#include <gtest/gtest.h>

#include "ferrari_control/trajectory_utils.hpp"

namespace utils = ferrari_control::trajectory_utils;
using ferrari_planning::msg::Trajectory;
using ferrari_planning::msg::TrajectoryPoint;

namespace
{
    TrajectoryPoint makePoint(double x, double y, double yaw, double v, double t)
    {
        TrajectoryPoint p;
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.orientation.z = std::sin(yaw / 2.0);
        p.pose.orientation.w = std::cos(yaw / 2.0);
        p.velocity.linear.x = v;
        p.time_from_start.sec = static_cast<int32_t>(std::floor(t));
        p.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
        return p;
    }

    Trajectory makeStraightTrajectory(int n, double spacing, double speed)
    {
        Trajectory traj;
        for (int i = 0; i < n; ++i)
        {
            traj.points.push_back(makePoint(i * spacing, 0.0, 0.0, speed, i * spacing / speed));
        }
        return traj;
    }

    geometry_msgs::msg::Pose makePose(double x, double y, double yaw)
    {
        return makePoint(x, y, yaw, 0.0, 0.0).pose;
    }
}

TEST(TrajectoryUtils, FrenetErrorOnStraightLine)
{
    Trajectory aligned;
    aligned.points.push_back(makePoint(1.0, 0.0, 0.0, 1.0, 0.0));

    auto err = utils::computeFrenetError(makePose(1.5, 0.1, 0.2), aligned);

    EXPECT_NEAR(err.s, 0.5, 1e-9);
    EXPECT_NEAR(err.d, 0.1, 1e-9);
    EXPECT_NEAR(err.mu, 0.2, 1e-9);
}

TEST(TrajectoryUtils, FrenetErrorWrapsHeading)
{
    Trajectory aligned;
    aligned.points.push_back(makePoint(0.0, 0.0, M_PI - 0.1, 1.0, 0.0));

    auto err = utils::computeFrenetError(makePose(0.0, 0.0, -M_PI + 0.1), aligned);

    EXPECT_NEAR(err.mu, 0.2, 1e-9);
}

TEST(TrajectoryUtils, FrenetErrorOnEmptyTrajectory)
{
    auto err = utils::computeFrenetError(makePose(1.0, 2.0, 0.3), Trajectory{});

    EXPECT_DOUBLE_EQ(err.s, 0.0);
    EXPECT_DOUBLE_EQ(err.d, 0.0);
    EXPECT_DOUBLE_EQ(err.mu, 0.0);
}

TEST(TrajectoryUtils, HorizonStartsFromNearestPoint)
{
    auto traj = makeStraightTrajectory(20, 0.1, 1.0);

    auto horizon = utils::getTrajectoryHorizonBySpace(makePose(0.52, 0.05, 0.0), traj, 4);

    ASSERT_EQ(horizon.points.size(), 4u);
    EXPECT_NEAR(horizon.points.front().pose.position.x, 0.5, 1e-9);
    EXPECT_NEAR(horizon.points.back().pose.position.x, 0.8, 1e-9);
}

TEST(TrajectoryUtils, HorizonIsTruncatedAtTrajectoryEnd)
{
    auto traj = makeStraightTrajectory(10, 0.1, 1.0);

    auto horizon = utils::getTrajectoryHorizonBySpace(makePose(0.88, 0.0, 0.0), traj, 5);

    EXPECT_EQ(horizon.points.size(), 1u);
}

TEST(TrajectoryUtils, HorizonOfEmptyTrajectoryIsEmpty)
{
    EXPECT_TRUE(utils::getTrajectoryHorizonBySpace(makePose(0, 0, 0), Trajectory{}, 5).points.empty());
}

TEST(TrajectoryUtils, ResampledHorizonFollowsTime)
{
    auto traj = makeStraightTrajectory(11, 0.1, 1.0);

    auto horizon = utils::getResampledSpatioTemporalHorizon(traj, makePose(0.0, 0.0, 0.0), 0.25, 4);

    ASSERT_EQ(horizon.size(), 4u);
    for (size_t k = 0; k < horizon.size(); ++k)
    {
        EXPECT_NEAR(horizon[k].pose.position.x, 0.25 * k, 1e-6);
        EXPECT_NEAR(horizon[k].velocity.linear.x, 1.0, 1e-9);
    }
}

TEST(TrajectoryUtils, ResampledHorizonPadsWithLastPoint)
{
    auto traj = makeStraightTrajectory(3, 0.1, 1.0);

    auto horizon = utils::getResampledSpatioTemporalHorizon(traj, makePose(0.0, 0.0, 0.0), 0.5, 4);

    ASSERT_EQ(horizon.size(), 4u);
    EXPECT_NEAR(horizon.back().pose.position.x, 0.2, 1e-9);
}

TEST(TrajectoryUtils, InterpolationTakesShortestYawPath)
{
    auto a = makePoint(0.0, 0.0, M_PI - 0.1, 1.0, 0.0);
    auto b = makePoint(1.0, 0.0, -M_PI + 0.1, 3.0, 1.0);

    auto mid = utils::interpolatePoints(a, b, 0.5);

    EXPECT_NEAR(mid.pose.position.x, 0.5, 1e-9);
    EXPECT_NEAR(mid.velocity.linear.x, 2.0, 1e-9);
    EXPECT_NEAR(std::abs(std::remainder(2.0 * std::atan2(mid.pose.orientation.z, mid.pose.orientation.w) - M_PI, 2.0 * M_PI)), 0.0, 1e-9);
}

TEST(TrajectoryUtils, LookaheadPointIsExpressedInVehicleFrame)
{
    auto traj = makeStraightTrajectory(50, 0.1, 1.0);

    auto target = utils::getLookaheadPoint(traj, makePose(0.0, 0.0, M_PI / 2.0), 0.5);

    EXPECT_NEAR(target.x, 0.0, 1e-6);
    EXPECT_NEAR(target.y, -0.5, 1e-6);
}

TEST(TrajectoryUtils, TargetVelocityIsTakenFromNearestPoint)
{
    Trajectory traj;
    traj.points.push_back(makePoint(0.0, 0.0, 0.0, 0.3, 0.0));
    traj.points.push_back(makePoint(1.0, 0.0, 0.0, 0.9, 1.0));

    EXPECT_NEAR(utils::getTargetVelocity(traj, makePose(0.9, 0.0, 0.0)), 0.9, 1e-9);
    EXPECT_DOUBLE_EQ(utils::getTargetVelocity(Trajectory{}, makePose(0.0, 0.0, 0.0)), 0.0);
}
