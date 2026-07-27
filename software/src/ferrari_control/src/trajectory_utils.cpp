#include "ferrari_control/trajectory_utils.hpp"
#include <limits>

namespace ferrari_control::trajectory_utils
{
    inline double durationToSec(const builtin_interfaces::msg::Duration &d)
    {
        return d.sec + d.nanosec * 1e-9;
    }

    static double getYawFromPose(const geometry_msgs::msg::Pose &pose)
    {
        double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    FrenetError computeFrenetError(
        const geometry_msgs::msg::Pose &pose,
        const ferrari_planning::msg::Trajectory &aligned_trajectory)
    {
        FrenetError err{0.0, 0.0, 0.0};
        if (aligned_trajectory.points.empty())
        {
            return err;
        }

        const auto &ref_point = aligned_trajectory.points.front();
        double ref_yaw = getYawFromPose(ref_point.pose);
        double current_yaw = getYawFromPose(pose);

        double dx = pose.position.x - ref_point.pose.position.x;
        double dy = pose.position.y - ref_point.pose.position.y;

        // Project Cartesian errors into the Frenet frame
        err.s = dx * std::cos(ref_yaw) + dy * std::sin(ref_yaw);
        err.d = -dx * std::sin(ref_yaw) + dy * std::cos(ref_yaw);
        err.mu = normalizeAngle(current_yaw - ref_yaw);

        return err;
    }

    ferrari_planning::msg::Trajectory getTrajectoryHorizonBySpace(
        const geometry_msgs::msg::Pose &pose,
        const ferrari_planning::msg::Trajectory &trajectory,
        int horizon_steps)
    {
        ferrari_planning::msg::Trajectory horizon;
        horizon.header = trajectory.header;

        if (trajectory.points.empty())
            return horizon;

        size_t nearest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();

        for (size_t i = 0; i < trajectory.points.size(); ++i)
        {
            double dx = pose.position.x - trajectory.points[i].pose.position.x;
            double dy = pose.position.y - trajectory.points[i].pose.position.y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                nearest_idx = i;
            }
        }

        size_t end_idx = std::min(trajectory.points.size(), nearest_idx + horizon_steps);
        horizon.points.assign(
            trajectory.points.begin() + nearest_idx,
            trajectory.points.begin() + end_idx);

        return horizon;
    }

    std::vector<ferrari_planning::msg::TrajectoryPoint> getResampledSpatioTemporalHorizon(
        const ferrari_planning::msg::Trajectory &trajectory,
        const geometry_msgs::msg::Pose &current_pose,
        double dt,
        int horizon_steps)
    {
        std::vector<ferrari_planning::msg::TrajectoryPoint> horizon;

        if (trajectory.points.empty())
        {
            return horizon;
        }
        if (trajectory.points.size() == 1)
        {
            horizon.assign(horizon_steps, trajectory.points.front());
            return horizon;
        }

        // Step 1: Spatial Anchoring - Find the closest geographical point
        size_t nearest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();

        for (size_t i = 0; i < trajectory.points.size(); ++i)
        {
            double dx = current_pose.position.x - trajectory.points[i].pose.position.x;
            double dy = current_pose.position.y - trajectory.points[i].pose.position.y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                nearest_idx = i;
            }
        }

        // Step 2: Temporal Synchronization - Get the time of the closest point
        double t_start = durationToSec(trajectory.points[nearest_idx].time_from_start);

        // Step 3: Resample using the MPC's dt
        double target_time = 0.0;
        size_t current_search_idx = nearest_idx;

        for (int step = 0; step < horizon_steps; ++step)
        {
            target_time = t_start + (step * dt);

            bool found = false;

            // Slide the search index forward to find the segment containing target_time
            while (current_search_idx < trajectory.points.size() - 1)
            {
                double t1 = durationToSec(trajectory.points[current_search_idx].time_from_start);
                double t2 = durationToSec(trajectory.points[current_search_idx + 1].time_from_start);

                // Handle potential edge case where trajectory planner stacks timestamps
                if (t2 <= t1)
                {
                    current_search_idx++;
                    continue;
                }

                if (target_time >= t1 && target_time <= t2)
                {
                    double alpha = (target_time - t1) / (t2 - t1);
                    horizon.push_back(interpolatePoints(trajectory.points[current_search_idx], trajectory.points[current_search_idx + 1], alpha));
                    found = true;
                    break;
                }
                else if (target_time > t2)
                {
                    // Target time is further ahead, move to the next segment
                    current_search_idx++;
                }
                else
                {
                    // Target time is somehow behind (shouldn't happen with positive dt), clamp to t1
                    horizon.push_back(trajectory.points[current_search_idx]);
                    found = true;
                    break;
                }
            }

            // Pad if the target time exceeds the end of the provided local trajectory
            if (!found)
            {
                horizon.push_back(trajectory.points.back());
            }
        }

        return horizon;
    }

    ferrari_planning::msg::TrajectoryPoint interpolatePoints(
        const ferrari_planning::msg::TrajectoryPoint &a,
        const ferrari_planning::msg::TrajectoryPoint &b,
        double alpha)
    {
        ferrari_planning::msg::TrajectoryPoint p;

        // Position
        p.pose.position.x = a.pose.position.x + alpha * (b.pose.position.x - a.pose.position.x);
        p.pose.position.y = a.pose.position.y + alpha * (b.pose.position.y - a.pose.position.y);

        // Orientation
        double yaw_a = getYawFromPose(a.pose);
        double yaw_b = getYawFromPose(b.pose);
        double yaw_diff = normalizeAngle(yaw_b - yaw_a);
        double yaw_p = normalizeAngle(yaw_a + alpha * yaw_diff);

        p.pose.orientation.w = std::cos(yaw_p * 0.5);
        p.pose.orientation.z = std::sin(yaw_p * 0.5);
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;

        // Velocity (assuming twist.linear.x or a dedicated velocity field)
        p.velocity.linear.x = a.velocity.linear.x + alpha * (b.velocity.linear.x - a.velocity.linear.x);
        p.velocity.linear.y = a.velocity.linear.y + alpha * (b.velocity.linear.y - a.velocity.linear.y);
        p.velocity.linear.z = a.velocity.linear.z + alpha * (b.velocity.linear.z - a.velocity.linear.z);
        p.velocity.angular.x = a.velocity.angular.x + alpha * (b.velocity.angular.x - a.velocity.angular.x);
        p.velocity.angular.y = a.velocity.angular.y + alpha * (b.velocity.angular.y - a.velocity.angular.y);
        p.velocity.angular.z = a.velocity.angular.z + alpha * (b.velocity.angular.z - a.velocity.angular.z);
        p.time_from_start = a.time_from_start; // Simplified handling of timestamp

        return p;
    }

    double calculateCrossTrackError(const ferrari_planning::msg::Trajectory &trajectory, const geometry_msgs::msg::Pose &pose)
    {
        auto aligned = getTrajectoryHorizonBySpace(pose, trajectory, 2);
        return computeFrenetError(pose, aligned).d;
    }

    double calculateHeadingError(const ferrari_planning::msg::Trajectory &trajectory, const geometry_msgs::msg::Pose &pose)
    {
        auto aligned = getTrajectoryHorizonBySpace(pose, trajectory, 2);
        return computeFrenetError(pose, aligned).mu;
    }

    double getTargetVelocity(const ferrari_planning::msg::Trajectory &trajectory, const geometry_msgs::msg::Pose &pose)
    {
        auto aligned = getTrajectoryHorizonBySpace(pose, trajectory, 2);
        if (aligned.points.empty())
            return 0.0;

        // Returns the reference speed of the closest trajectory point
        return aligned.points.front().velocity.linear.x;
    }

    geometry_msgs::msg::Point getLookaheadPoint(
        const ferrari_planning::msg::Trajectory &trajectory,
        const geometry_msgs::msg::Pose &pose,
        double lookahead_distance)
    {
        geometry_msgs::msg::Point local_target;

        auto horizon = getTrajectoryHorizonBySpace(pose, trajectory, 100);
        if (horizon.points.empty())
            return local_target;

        // Default to the last point in case the path is shorter than lookahead_distance
        size_t target_idx = horizon.points.size() - 1;
        for (size_t i = 0; i < horizon.points.size(); ++i)
        {
            double dx = horizon.points[i].pose.position.x - pose.position.x;
            double dy = horizon.points[i].pose.position.y - pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist >= lookahead_distance)
            {
                target_idx = i;
                break;
            }
        }

        double target_x = horizon.points[target_idx].pose.position.x;
        double target_y = horizon.points[target_idx].pose.position.y;

        double car_yaw = getYawFromPose(pose);
        double dx = target_x - pose.position.x;
        double dy = target_y - pose.position.y;

        // Rotate into base_link local frame
        local_target.x = dx * std::cos(-car_yaw) - dy * std::sin(-car_yaw);
        local_target.y = dx * std::sin(-car_yaw) + dy * std::cos(-car_yaw);
        local_target.z = 0.0;

        return local_target;
    }

} // namespace ferrari_control::trajectory_utils