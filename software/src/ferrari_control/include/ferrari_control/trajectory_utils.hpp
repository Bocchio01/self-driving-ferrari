#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ferrari_planning/msg/trajectory.hpp>
#include <ferrari_planning/msg/trajectory_point.hpp>

namespace ferrari_control::trajectory_utils
{
    struct FrenetError
    {
        double s;
        double d;
        double mu;
    };

    /**
     * @brief Computes Frenet frame errors assuming the first point of the trajectory is the origin.
     */
    FrenetError computeFrenetError(
        const geometry_msgs::msg::Pose &pose,
        const ferrari_planning::msg::Trajectory &aligned_trajectory);

    /**
     * @brief Extracts a spatially-aligned trajectory horizon starting from the nearest physical point.
     */
    ferrari_planning::msg::Trajectory getTrajectoryHorizonBySpace(
        const geometry_msgs::msg::Pose &pose,
        const ferrari_planning::msg::Trajectory &trajectory,
        int horizon_steps);

    /**
     * @brief Extracts a temporally-aligned horizon of trajectory points based on elapsed time.
     */
    std::vector<ferrari_planning::msg::TrajectoryPoint> getResampledSpatioTemporalHorizon(
        const ferrari_planning::msg::Trajectory &trajectory,
        const geometry_msgs::msg::Pose &current_pose,
        double dt,
        int horizon_steps);

    /**
     * @brief Linearly interpolates between two trajectory points.
     */
    ferrari_planning::msg::TrajectoryPoint interpolatePoints(
        const ferrari_planning::msg::TrajectoryPoint &a,
        const ferrari_planning::msg::TrajectoryPoint &b,
        double alpha);

    double calculateCrossTrackError(const ferrari_planning::msg::Trajectory &trajectory, const geometry_msgs::msg::Pose &pose);
    double calculateHeadingError(const ferrari_planning::msg::Trajectory &trajectory, const geometry_msgs::msg::Pose &pose);
    double getTargetVelocity(const ferrari_planning::msg::Trajectory &trajectory, const geometry_msgs::msg::Pose &pose);
    geometry_msgs::msg::Point getLookaheadPoint(const ferrari_planning::msg::Trajectory &trajectory, const geometry_msgs::msg::Pose &pose, double lookahead_distance);

} // namespace ferrari_control::trajectory_utils