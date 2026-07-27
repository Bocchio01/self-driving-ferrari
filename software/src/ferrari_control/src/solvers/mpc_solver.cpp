#include <algorithm>

#include "ferrari_control/solvers/mpc_solver.hpp"

MpcSolver::MpcSolver(int nx, int nu, int prediction_horizon, int control_horizon)
    : nx_(nx),
      nu_(nu),
      NP_(prediction_horizon),
      NC_(control_horizon),
      initialized_(false)
{
    num_variables_ = nx_ * (NP_ + 1) + nu_ * NC_;
    num_dynamics_constraints_ = nx_ * (NP_ + 1);
    num_constraints_ = num_dynamics_constraints_ + nu_ * NC_;
}

void MpcSolver::setCosts(const Eigen::VectorXd &Q, const Eigen::VectorXd &R, const Eigen::VectorXd &QN, const Eigen::VectorXd &S)
{
    Q_ = Q;
    R_ = R;
    QN_ = QN;
    S_ = S;
}

void MpcSolver::setBounds(const Eigen::VectorXd &x_min, const Eigen::VectorXd &x_max,
                          const Eigen::VectorXd &u_min, const Eigen::VectorXd &u_max)
{
    x_min_ = x_min;
    x_max_ = x_max;
    u_min_ = u_min;
    u_max_ = u_max;
}

bool MpcSolver::initialize()
{
    buildHessian();

    gradient_ = Eigen::VectorXd::Zero(num_variables_);
    lower_bound_ = Eigen::VectorXd::Zero(num_constraints_);
    upper_bound_ = Eigen::VectorXd::Zero(num_constraints_);

    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(num_variables_);
    solver_.data()->setNumberOfConstraints(num_constraints_);

    if (!solver_.data()->setHessianMatrix(hessian_))
        return false;
    if (!solver_.data()->setGradient(gradient_))
        return false;

    // Dummy constraints for initialization, updated during solve()
    Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Identity(nx_, nx_);
    Eigen::MatrixXd B_dummy = Eigen::MatrixXd::Zero(nx_, nu_);
    buildConstraintMatrix(A_dummy, B_dummy);

    if (!solver_.data()->setLinearConstraintsMatrix(constraint_matrix_))
        return false;
    if (!solver_.data()->setLowerBound(lower_bound_))
        return false;
    if (!solver_.data()->setUpperBound(upper_bound_))
        return false;

    initialized_ = solver_.initSolver();
    return initialized_;
}

bool MpcSolver::solve(const Eigen::VectorXd &x0, const std::vector<Eigen::VectorXd> &x_ref,
                      const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::VectorXd &C,
                      const Eigen::VectorXd &u_prev, Eigen::VectorXd &u_opt)
{
    if (!initialized_)
        return false;

    buildConstraintMatrix(A, B);
    updateBounds(x0, C);
    updateGradient(x_ref, u_prev);

    solver_.updateLinearConstraintsMatrix(constraint_matrix_);
    solver_.updateBounds(lower_bound_, upper_bound_);
    solver_.updateGradient(gradient_);

    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return false;

    Eigen::VectorXd solution = solver_.getSolution();
    u_opt = solution.segment(nx_ * (NP_ + 1), nu_);
    return true;
}

void MpcSolver::buildHessian()
{
    std::vector<Eigen::Triplet<double>> triplets;
    // triplets.reserve(nx_ * (NP_ + 1) + nu_ * NC_);

    // State Costs (Q) + Terminal Cost (QN)
    for (int k = 0; k <= NP_; ++k)
    {
        int idx = k * nx_;
        for (int i = 0; i < nx_; ++i)
            triplets.emplace_back(idx + i, idx + i, (k == NP_) ? QN_(i) : Q_(i));
    }

    // Control Costs (R) + Control Rate Costs (S)
    int offset = nx_ * (NP_ + 1);
    for (int k = 0; k < NC_; ++k)
    {
        int idx = offset + k * nu_;
        for (int i = 0; i < nu_; ++i)
        {
            // Base effort cost
            double diag_cost = R_(i);

            // Add variation cost (S). It is added twice for intermediate steps.
            diag_cost += S_(i); // from (u_k - u_{k-1})^2
            if (k < NC_ - 1)
                diag_cost += S_(i); // from (u_{k+1} - u_k)^2

            triplets.emplace_back(idx + i, idx + i, diag_cost);

            // Off-diagonal coupling for -2 * u_k * u_{k-1}
            if (k > 0)
            {
                int prev_idx = offset + (k - 1) * nu_;
                triplets.emplace_back(idx + i, prev_idx + i, -S_(i));
                triplets.emplace_back(prev_idx + i, idx + i, -S_(i)); // Ensure symmetry
            }
        }
    }

    hessian_.resize(num_variables_, num_variables_);
    hessian_.setFromTriplets(triplets.begin(), triplets.end());
}

void MpcSolver::buildConstraintMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
{
    std::vector<Eigen::Triplet<double>> triplets;

    // Dynamics Constraints: x_{k+1} - A * x_k - B * u_k = 0
    for (int i = 0; i < nx_; ++i)
        triplets.emplace_back(i, i, 1.0);

    int control_offset = nx_ * (NP_ + 1);
    for (int k = 1; k <= NP_; ++k)
    {
        int row = k * nx_;
        int prev_state_col = (k - 1) * nx_;
        int state_col = k * nx_;

        // CLAMP the control index so it never exceeds NC_ - 1
        int active_k = std::min(k - 1, NC_ - 1);
        int control_col = control_offset + active_k * nu_;

        for (int i = 0; i < nx_; ++i)
        {
            triplets.emplace_back(row + i, state_col + i, 1.0);
            for (int j = 0; j < nx_; ++j)
                triplets.emplace_back(row + i, prev_state_col + j, -A(i, j));
            for (int j = 0; j < nu_; ++j)
                triplets.emplace_back(row + i, control_col + j, -B(i, j));
        }
    }

    int bounds_row_offset = num_dynamics_constraints_;
    for (int k = 0; k < NC_; ++k)
    {
        int row = bounds_row_offset + k * nu_;
        int col = control_offset + k * nu_;
        for (int i = 0; i < nu_; ++i)
            triplets.emplace_back(row + i, col + i, 1.0);
    }

    constraint_matrix_.resize(num_constraints_, num_variables_);
    constraint_matrix_.setFromTriplets(triplets.begin(), triplets.end());
}

// void MpcSolver::updateBounds(const Eigen::VectorXd &x0)
void MpcSolver::updateBounds(const Eigen::VectorXd &x0, const Eigen::VectorXd &C)
{
    lower_bound_.segment(0, nx_) = x0;
    upper_bound_.segment(0, nx_) = x0;

    for (int k = 1; k <= NP_; ++k)
    {
        // lower_bound_.segment(k * nx_, nx_).setZero();
        // upper_bound_.segment(k * nx_, nx_).setZero();
        lower_bound_.segment(k * nx_, nx_) = C;
        upper_bound_.segment(k * nx_, nx_) = C;
    }

    int bounds_row_offset = num_dynamics_constraints_;
    for (int k = 0; k < NC_; ++k)
    {
        lower_bound_.segment(bounds_row_offset + k * nu_, nu_) = u_min_;
        upper_bound_.segment(bounds_row_offset + k * nu_, nu_) = u_max_;
    }
}

void MpcSolver::updateGradient(const std::vector<Eigen::VectorXd> &x_ref, const Eigen::VectorXd &u_prev)
{
    gradient_.setZero(); // Ensure a clean slate

    // State reference tracking (same as before)
    for (int k = 0; k <= NP_; ++k)
    {
        const auto &ref = x_ref[std::min<std::size_t>(k, x_ref.size() - 1)];
        int idx = k * nx_;
        Eigen::VectorXd q_k = (k == NP_) ? QN_ : Q_;
        gradient_.segment(idx, nx_) = -q_k.cwiseProduct(ref);
    }

    // Control rate penalty for the FIRST step (k=0) interacting with u_prev
    int control_offset = nx_ * (NP_ + 1);
    for (int i = 0; i < nu_; ++i)
    {
        // -S * u_prev shifts the optimal u_0 towards u_prev
        gradient_(control_offset + i) = -S_(i) * u_prev(i);
    }
}