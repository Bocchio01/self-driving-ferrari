#include <algorithm>
#include <mutex>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "ferrari_control/trajectory_utils.hpp"
#include "ferrari_control/controllers/lateral_mpc_controller.hpp"

using namespace vehicle_models;
using namespace vehicle_models::ackermann;

LateralMpcControllerNode::LateralMpcControllerNode(const rclcpp::NodeOptions &options)
    : Controller("lateral_mpc_cmd", "lateral_mpc_controller_node", options),
      frenet_model_(kinematic_frenet::Params{})
{
    Np_ = this->declare_parameter<int>("prediction_horizon", 20);
    Nc_ = this->declare_parameter<int>("control_horizon", 5);

    this->declare_parameter<std::vector<double>>("Q", {10.0, 5.0}); // States: [d, mu]
    this->declare_parameter<std::vector<double>>("QN", {15.0, 10.0});
    this->declare_parameter<std::vector<double>>("R", {1.0}); // Control: [steering_angle]
    this->declare_parameter<std::vector<double>>("S", {0.0});
    this->declare_parameter<std::vector<double>>("state_min", {-10.0, -M_PI});
    this->declare_parameter<std::vector<double>>("state_max", {10.0, M_PI});
}

LifecycleNodeInterface::CallbackReturn LateralMpcControllerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    auto ret = Controller::on_configure(state);
    if (ret != LifecycleNodeInterface::CallbackReturn::SUCCESS)
        return ret;

    // Initialize the vehicle model (Frenet kinematic bicycle model)
    kinematic_frenet::Params params;
    params.wheelbase = wheelbase_;
    params.max_speed = max_speed_;
    params.max_steer = max_steering_angle_;
    frenet_model_ = kinematic_frenet::Model(params);

    // Fetch MPC parameters
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

    // Actuator limits (steering only)
    Eigen::VectorXd u_min(1);
    Eigen::VectorXd u_max(1);
    u_min << -max_steering_angle_;
    u_max << +max_steering_angle_;

    // MPC solver initialization
    mpc_solver_ = std::make_unique<MpcSolver>(2, 1, Np_, Nc_);
    mpc_solver_->setCosts(Q, R, QN, S);
    mpc_solver_->setBounds(s_min, s_max, u_min, u_max);

    if (!mpc_solver_->initialize())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to init OSQP workspace.");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    last_cmd_ = {0.0, 0.0};
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LateralMpcControllerNode::computeControlCommand()
{
    if (!is_active_)
        return;

    ferrari_planning::msg::Trajectory current_trajectory;
    {
        std::lock_guard<std::mutex> lock(local_trajectory_mutex_);
        if (local_trajectory_.points.empty())
            return;
        current_trajectory = local_trajectory_;
    }

    // 1. Spatial alignment using the utility namespace
    ferrari_planning::msg::Trajectory spatial_traj = ferrari_control::trajectory_utils::getTrajectoryHorizonBySpace(odom_.pose.pose, current_trajectory, Np_);

    // 2. Pure geometry math
    ferrari_control::trajectory_utils::FrenetError err = ferrari_control::trajectory_utils::computeFrenetError(odom_.pose.pose, spatial_traj);

    // 3. Map to specific vehicle model state
    kinematic_frenet::State current_frenet{err.s, err.d, err.mu};

    // 4. Temporal lookup for speed
    auto time_horizon = ferrari_control::trajectory_utils::getResampledSpatioTemporalHorizon(current_trajectory, odom_.pose.pose, control_period_.count(), 1);
    double v_ref = time_horizon.empty() ? spatial_traj.points[0].velocity.linear.x : time_horizon[0].velocity.linear.x;

    // Full model linearization and successive reduction for lateral-only control
    auto x0_full = kinematic_frenet::Model::toVector(current_frenet);

    // We need v_ref purely to linearize the A and B matrices correctly for the current speed
    auto u0_full = kinematic_frenet::Model::toVector(kinematic_frenet::Command{v_ref, last_cmd_.steering_angle});

    kinematic_frenet::Model::StateJacobian A_full;
    kinematic_frenet::Model::ControlJacobian B_full;
    frenet_model_.getLinearized(x0_full, u0_full, control_period_.count(), A_full, B_full);

    Eigen::VectorXd x_next_nominal = frenet_model_.stepImpl(x0_full, u0_full, control_period_.count());
    Eigen::VectorXd C_full = x_next_nominal - (A_full * x0_full) - (B_full * u0_full);

    Eigen::MatrixXd A(2, 2);
    Eigen::MatrixXd B(2, 1);
    Eigen::VectorXd C(2);
    B << B_full(1, 1),
        B_full(2, 1);
    A << A_full(1, 1), A_full(1, 2),
        A_full(2, 1), A_full(2, 2);
    C << C_full(1), C_full(2);

    // Frenet reference trajectory: d = 0, mu = 0
    std::vector<Eigen::VectorXd> x_ref;
    for (int k = 0; k <= Np_; ++k)
    {
        Eigen::VectorXd ref(2);
        ref << 0.0, 0.0;
        x_ref.push_back(ref);
    }

    // Set up the initial state and previous control input for the MPC solver
    Eigen::VectorXd x0(2);
    x0 << current_frenet.d, current_frenet.mu;

    Eigen::VectorXd u_prev(1);
    u_prev << last_cmd_.steering_angle;

    Eigen::VectorXd u_opt(1);

    // Solve the MPC problem
    if (mpc_solver_->solve(x0, x_ref, A, B, C, u_prev, u_opt))
    {
        last_cmd_.steering_angle = u_opt(0);

        ackermann_msgs::msg::AckermannDriveStamped cmd_msg;
        cmd_msg.drive.steering_angle = last_cmd_.steering_angle;

        // Speed is ignored entirely; the longitudinal PID handles velocity tracking
        // cmd_msg.drive.speed = 0.0;
        cmd_msg.drive.speed = v_ref;

        publishCmd(cmd_msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "MPC infeasible! Holding last command.");
    }
}

rcl_interfaces::msg::SetParametersResult LateralMpcControllerNode::onParamsChanged(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    // If the solver hasn't been initialized yet, just accept the parameters
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
        // Safety check for Frenet model dimensions
        if (q_param.size() != 2 || qn_param.size() != 2 || r_param.size() != 1 || s_param.size() != 1)
        {
            result.successful = false;
            result.reason = "Invalid parameter array size. Q/QN must be 2, R/S must be 1.";
            return result;
        }

        Eigen::VectorXd Q = Eigen::Map<Eigen::VectorXd>(q_param.data(), q_param.size());
        Eigen::VectorXd QN = Eigen::Map<Eigen::VectorXd>(qn_param.data(), qn_param.size());
        Eigen::VectorXd R = Eigen::Map<Eigen::VectorXd>(r_param.data(), r_param.size());
        Eigen::VectorXd S = Eigen::Map<Eigen::VectorXd>(s_param.data(), s_param.size());

        mpc_solver_->setCosts(Q, R, QN, S);
        RCLCPP_INFO(this->get_logger(), "Lateral MPC cost parameters updated dynamically.");
    }

    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<LateralMpcControllerNode> node = std::make_shared<LateralMpcControllerNode>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}