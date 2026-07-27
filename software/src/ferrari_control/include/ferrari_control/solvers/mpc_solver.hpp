#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>

class MpcSolver
{
public:
    MpcSolver(int nx, int nu, int prediction_horizon, int control_horizon);

    void setCosts(const Eigen::VectorXd &Q, const Eigen::VectorXd &R,
                  const Eigen::VectorXd &Q_terminal, const Eigen::VectorXd &S);

    void setBounds(const Eigen::VectorXd &x_min, const Eigen::VectorXd &x_max,
                   const Eigen::VectorXd &u_min, const Eigen::VectorXd &u_max);

    bool initialize();

    bool solve(const Eigen::VectorXd &x0, const std::vector<Eigen::VectorXd> &x_ref,
               const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::VectorXd &C,
               const Eigen::VectorXd &u_prev, Eigen::VectorXd &u_opt);

private:
    void buildHessian();
    void buildConstraintMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    void updateGradient(const std::vector<Eigen::VectorXd> &x_ref, const Eigen::VectorXd &u_prev);
    void updateBounds(const Eigen::VectorXd &x0, const Eigen::VectorXd &C);

    int nx_, nu_, NP_, NC_;
    int num_variables_, num_constraints_, num_dynamics_constraints_;

    Eigen::VectorXd Q_, R_, QN_, S_;
    Eigen::VectorXd x_min_, x_max_, u_min_, u_max_;

    OsqpEigen::Solver solver_;
    bool initialized_;

    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    Eigen::SparseMatrix<double> constraint_matrix_;
    Eigen::VectorXd lower_bound_, upper_bound_;
};