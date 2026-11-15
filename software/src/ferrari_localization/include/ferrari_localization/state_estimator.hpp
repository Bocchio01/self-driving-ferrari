#pragma once

#include <cstddef>
#include <functional>

#include <Eigen/Dense>

#include "ferrari_localization/extended_kalman_filter.hpp"
#include "ferrari_localization/particle_filter.hpp"

namespace ferrari_localization
{
    enum StateIndex
    {
        X = 0,
        Y = 1,
        YAW = 2,
        SPEED = 3,
        GYRO_BIAS = 4,
        STATE_DIM = 5
    };

    using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
    using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

    struct NoiseDensities
    {
        double position = 0.01;
        double heading = 0.01;
        double speed = 1.0;
        double gyro_bias = 0.001;
    };

    struct InitialUncertainty
    {
        double pose = 0.01;
        double speed = 0.3;
        double gyro_bias = 0.1;
    };

    struct Estimate
    {
        StateVector state = StateVector::Zero();
        StateMatrix covariance = StateMatrix::Zero();
        double yaw_rate = 0.0;
    };

    StateVector propagateMotion(const StateVector &x, double gyro_z, double dt, StateMatrix *jacobian = nullptr);

    class StateEstimator
    {
    public:
        virtual ~StateEstimator() = default;

        virtual void reset() = 0;
        virtual void predict(double gyro_z, double dt) = 0;
        virtual void updateWheelSpeed(double speed, double variance) = 0;
        virtual void updateYawRate(double yaw_rate, double variance) = 0;
        virtual void updateGyroBias(double gyro_z, double variance) = 0;
        virtual Estimate estimate() const = 0;
    };

    class EkfEstimator : public StateEstimator
    {
    public:
        EkfEstimator(const NoiseDensities &noise, const InitialUncertainty &initial);

        void reset() override;
        void predict(double gyro_z, double dt) override;
        void updateWheelSpeed(double speed, double variance) override;
        void updateYawRate(double yaw_rate, double variance) override;
        void updateGyroBias(double gyro_z, double variance) override;
        Estimate estimate() const override;

    private:
        NoiseDensities noise_;
        InitialUncertainty initial_;
        ExtendedKalmanFilter filter_;
        double last_gyro_ = 0.0;
    };

    class ParticleEstimator : public StateEstimator
    {
    public:
        ParticleEstimator(size_t particle_count, const NoiseDensities &noise, const InitialUncertainty &initial, unsigned int seed = 0);

        void reset() override;
        void predict(double gyro_z, double dt) override;
        void updateWheelSpeed(double speed, double variance) override;
        void updateYawRate(double yaw_rate, double variance) override;
        void updateGyroBias(double gyro_z, double variance) override;
        Estimate estimate() const override;

    private:
        void ensembleUpdate(double measurement, double variance, const std::function<double(const Eigen::VectorXd &)> &predicted_measurement);

        size_t particle_count_;
        NoiseDensities noise_;
        InitialUncertainty initial_;
        ParticleFilter filter_;
        double last_gyro_ = 0.0;
    };
}
