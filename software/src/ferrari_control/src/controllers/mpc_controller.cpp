#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "ferrari_control/controllers/mpc_controller.hpp"
#include "ferrari_control/trajectory_utils.hpp"

using namespace vehicle_models;
using namespace vehicle_models::ackermann;

namespace
{
    constexpr double kDefaultGravity = 9.81;
} // namespace

MpcControllerNode::MpcControllerNode(const rclcpp::NodeOptions &options)
    : Controller("mpc_cmd", "mpc_controller_node", options),
      model_(dynamic_cartesian::Params{})
{
    Np_ = this->declare_parameter<int>("prediction_horizon", 15);
    Nc_ = this->declare_parameter<int>("control_horizon", 5);
    friction_coefficient_ = this->declare_parameter<double>("friction_coefficient", 1.0);
    gravity_ = this->declare_parameter<double>("gravity", kDefaultGravity);

    // State: [x, y, yaw, vx, vy, yaw_rate]. Control: [acceleration, steering_angle].
    this->declare_parameter<std::vector<double>>("Q", {1.0, 1.0, 0.5, 5.0, 1.0, 0.5});
    this->declare_parameter<std::vector<double>>("QN", {2.0, 2.0, 1.0, 8.0, 2.0, 1.0});
    this->declare_parameter<std::vector<double>>("R", {0.1, 1.0});
    this->declare_parameter<std::vector<double>>("S", {0.05, 0.5});
    this->declare_parameter<std::vector<double>>("state_min", {-1e6, -1e6, -1e6, 0.0, -1e6, -1e6});
    this->declare_parameter<std::vector<double>>("state_max", {1e6, 1e6, 1e6, 1e6, 1e6, 1e6});
}

LifecycleNodeInterface::CallbackReturn MpcControllerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    auto ret = Controller::on_configure(state);
    if (ret != LifecycleNodeInterface::CallbackReturn::SUCCESS)
        return ret;

    dynamic_cartesian::Params params;
    params.wheelbase = wheelbase_;
    params.max_steer = max_steering_angle_;
    params.max_accel = max_longitudinal_acceleration_;
    model_ = dynamic_cartesian::Model(params);

    auto q_param = this->get_parameter("Q").as_double_array();
    auto qn_param = this->get_parameter("QN").as_double_array();
    auto r_param = this->get_parameter("R").as_double_array();
    auto s_param = this->get_parameter("S").as_double_array();
    auto s_min_param = this->get_parameter("state_min").as_double_array();
    auto s_max_param = this->get_parameter("state_max").as_double_array();

    Eigen::VectorXd Q = Eigen::Map<Eigen::VectorXd>(q_param.data(), q_param.size());
    Eigen::VectorXd QN = Eigen::Map<Eigen::VectorXd>(qn_param.data(), qn_param.size());
    Eigen::VectorXd R = Eigen::Map<Eigen::VectorXd>(r_param.data(), r_param.size());
    Eigen::VectorXd S = Eigen::Map<Eigen::VectorXd>(s_param.data(), s_param.size());
    Eigen::VectorXd s_min = Eigen::Map<Eigen::VectorXd>(s_min_param.data(), s_min_param.size());
    Eigen::VectorXd s_max = Eigen::Map<Eigen::VectorXd>(s_max_param.data(), s_max_param.size());

    s_min(3) = -1 / 4 * this->max_speed_;
    s_max(3) = +this->max_speed_;

    Eigen::VectorXd u_min(2), u_max(2);
    const double accel_limit = std::min(max_longitudinal_acceleration_, friction_coefficient_ * gravity_);
    u_min << -accel_limit, -max_steering_angle_;
    u_max << accel_limit, max_steering_angle_;

    mpc_solver_ = std::make_unique<MpcSolver>(6, 2, Np_, Nc_);
    mpc_solver_->setCosts(Q, R, QN, S);
    mpc_solver_->setBounds(s_min, s_max, u_min, u_max);

    if (!mpc_solver_->initialize())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to init OSQP workspace for dynamic MPC.");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    last_cmd_ = {0.0, 0.0};
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

dynamic_cartesian::State MpcControllerNode::computeCartesianState(const nav_msgs::msg::Odometry &odom) const
{
    dynamic_cartesian::State s;
    s.x = odom.pose.pose.position.x;
    s.y = odom.pose.pose.position.y;
    s.yaw = tf2::getYaw(odom.pose.pose.orientation);
    s.vx = odom.twist.twist.linear.x;
    s.vy = odom.twist.twist.linear.y;
    s.yaw_rate = odom.twist.twist.angular.z;
    return s;
}

std::vector<Eigen::VectorXd> MpcControllerNode::buildReferenceTrajectory(
    const dynamic_cartesian::State &current_state, const ferrari_planning::msg::Trajectory &trajectory) const
{
    std::vector<Eigen::VectorXd> x_ref;

    // 1. Ask the utility namespace for the temporally aligned raw points
    auto horizon_points = ferrari_control::trajectory_utils::getResampledSpatioTemporalHorizon(
        trajectory,
        odom_.pose.pose,
        control_period_.count(), // You MUST pass dt so the solver scales the distances properly
        Np_);

    // 2. Handle the fallback if no path exists
    if (horizon_points.empty())
    {
        for (int k = 0; k <= Np_; ++k)
        {
            Eigen::VectorXd ref(6);
            ref << current_state.x, current_state.y, current_state.yaw, 0.0, 0.0, 0.0;
            x_ref.push_back(ref);
        }
        return x_ref;
    }

    // 3. Map the generic TrajectoryPoints to this specific MPC's Eigen formulation
    for (const auto &pt : horizon_points)
    {
        double raw_yaw_ref = tf2::getYaw(pt.pose.orientation);
        double yaw_ref = current_state.yaw + std::remainder(raw_yaw_ref - current_state.yaw, 2.0 * M_PI);
        double v_ref = pt.velocity.linear.x;

        Eigen::VectorXd ref(6);
        ref << pt.pose.position.x, pt.pose.position.y, yaw_ref, v_ref, 0.0, 0.0;
        x_ref.push_back(ref);
    }

    return x_ref;
}

void MpcControllerNode::computeControlCommand()
{
    if (!is_active_)
        return;

    const dynamic_cartesian::State current_state = computeCartesianState(odom_);
    auto x0 = dynamic_cartesian::Model::toVector(current_state);
    auto u0 = dynamic_cartesian::Model::toVector(last_cmd_);

    dynamic_cartesian::Model::StateJacobian A;
    dynamic_cartesian::Model::ControlJacobian B;
    model_.getLinearized(x0, u0, control_period_.count(), A, B);

    // --- CALCULATE AFFINE TERM (C) ---
    // 1. Get the non-linear forward prediction f(x0, u0) using your RK4 stepper
    // 2. Compute C = f(x0, u0) - A*x0 - B*u0
    Eigen::VectorXd x_next_nominal = model_.stepImpl(x0, u0, control_period_.count());
    Eigen::VectorXd C = x_next_nominal - (A * x0) - (B * u0);

    // Safely copy the trajectory thread from the base class callback
    ferrari_planning::msg::Trajectory current_trajectory;
    {
        std::lock_guard<std::mutex> lock(local_trajectory_mutex_);
        current_trajectory = local_trajectory_;
    }

    auto x_ref = buildReferenceTrajectory(current_state, current_trajectory);

    Eigen::VectorXd u_opt(2);
    if (mpc_solver_->solve(x0, x_ref, A, B, C, u0, u_opt))
    {
        last_cmd_ = dynamic_cartesian::Model::ToStruct(u_opt);

        ackermann_msgs::msg::AckermannDriveStamped cmd_msg;
        cmd_msg.drive.acceleration = last_cmd_.acceleration;
        cmd_msg.drive.steering_angle = last_cmd_.steering_angle;
        cmd_msg.drive.speed = std::clamp(current_state.vx + last_cmd_.acceleration * control_period_.count(),
                                         0.0, max_speed_);
        publishCmd(cmd_msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Dynamic MPC infeasible! Sending emergency stop.");
        last_cmd_ = {-max_longitudinal_acceleration_, 0.0};

        ackermann_msgs::msg::AckermannDriveStamped cmd_msg;
        cmd_msg.drive.acceleration = -max_longitudinal_acceleration_;
        cmd_msg.drive.steering_angle = 0.0;
        cmd_msg.drive.speed = 0.0;
        publishCmd(cmd_msg);
    }
}

rcl_interfaces::msg::SetParametersResult MpcControllerNode::onParamsChanged(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    // If the solver hasn't been initialized yet (e.g., before on_configure),
    // just accept the parameters and let on_configure handle them later.
    if (!mpc_solver_)
    {
        return result;
    }

    bool update_costs = false;

    // 1. Fetch current values as the baseline
    auto q_param = this->get_parameter("Q").as_double_array();
    auto qn_param = this->get_parameter("QN").as_double_array();
    auto r_param = this->get_parameter("R").as_double_array();
    auto s_param = this->get_parameter("S").as_double_array();

    // 2. Override with any incoming changes
    for (const auto &param : parameters)
    {
        if (param.get_name() == "Q")
        {
            q_param = param.as_double_array();
            update_costs = true;
        }
        else if (param.get_name() == "QN")
        {
            qn_param = param.as_double_array();
            update_costs = true;
        }
        else if (param.get_name() == "R")
        {
            r_param = param.as_double_array();
            update_costs = true;
        }
        else if (param.get_name() == "S")
        {
            s_param = param.as_double_array();
            update_costs = true;
        }
    }

    // 3. Apply to the solver if necessary
    if (update_costs)
    {
        // Simple safety check for dimensions
        if (q_param.size() != 6 || qn_param.size() != 6 || r_param.size() != 2 || s_param.size() != 2)
        {
            result.successful = false;
            result.reason = "Invalid parameter array size. Q/QN must be 6, R/S must be 2.";
            return result;
        }

        Eigen::VectorXd Q = Eigen::Map<Eigen::VectorXd>(q_param.data(), q_param.size());
        Eigen::VectorXd QN = Eigen::Map<Eigen::VectorXd>(qn_param.data(), qn_param.size());
        Eigen::VectorXd R = Eigen::Map<Eigen::VectorXd>(r_param.data(), r_param.size());
        Eigen::VectorXd S = Eigen::Map<Eigen::VectorXd>(s_param.data(), s_param.size());

        mpc_solver_->setCosts(Q, R, QN, S);
        RCLCPP_INFO(this->get_logger(), "MPC cost parameters updated dynamically.");
    }

    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<MpcControllerNode> node = std::make_shared<MpcControllerNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}