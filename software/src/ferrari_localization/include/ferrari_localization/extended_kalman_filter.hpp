#pragma once

#include <Eigen/Dense>

namespace ferrari_localization
{
    class ExtendedKalmanFilter
    {
    public:
        void reset(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0)
        {
            x_ = x0;
            P_ = P0;
        }

        void predict(const Eigen::VectorXd &x_predicted, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q)
        {
            x_ = x_predicted;
            P_ = F * P_ * F.transpose() + Q;
            P_ = 0.5 * (P_ + P_.transpose());
        }

        double update(const Eigen::VectorXd &z, const Eigen::VectorXd &h, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R)
        {
            const Eigen::VectorXd innovation = z - h;
            const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
            const Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
            const Eigen::MatrixXd K = S_ldlt.solve(H * P_).transpose();

            x_ += K * innovation;

            const Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H;
            P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
            P_ = 0.5 * (P_ + P_.transpose());

            return innovation.dot(S_ldlt.solve(innovation));
        }

        const Eigen::VectorXd &state() const { return x_; }
        const Eigen::MatrixXd &covariance() const { return P_; }
        Eigen::VectorXd &mutableState() { return x_; }

    private:
        Eigen::VectorXd x_;
        Eigen::MatrixXd P_;
    };
}
