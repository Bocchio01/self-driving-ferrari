#pragma once

#include <cstddef>
#include <functional>
#include <vector>

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
        YAW_RATE = 4,
        GYRO_BIAS = 5,
        STATE_DIM = 6
    };

    using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
    using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

    struct NoiseDensities
    {
        double position = 0.01;
        double speed = 1.0;
        double yaw_rate = 1.0;
        double gyro_bias = 0.001;
    };

    struct InitialUncertainty
    {
        double pose = 0.01;
        double speed = 0.3;
        double yaw_rate = 0.5;
        double gyro_bias = 0.1;
    };

    struct Estimate
    {
        StateVector state = StateVector::Zero();
        StateMatrix covariance = StateMatrix::Zero();
    };

    StateVector propagateMotion(const StateVector &x, double dt, StateMatrix *jacobian = nullptr);

    class StateEstimator
    {
    public:
        virtual ~StateEstimator() = default;

        virtual void reset() = 0;
        virtual void predict(double dt) = 0;
        virtual void updateWheelSpeed(double speed, double variance) = 0;
        virtual void updateKinematicYawRate(double yaw_rate, double variance) = 0;
        virtual void updateGyro(double gyro_z, double variance) = 0;
        virtual void updateGyroBias(double gyro_z, double variance) = 0;
        virtual Estimate estimate() const = 0;
        virtual std::vector<StateVector> particles() const { return {}; }
    };

    class EkfEstimator : public StateEstimator
    {
    public:
        EkfEstimator(const NoiseDensities &noise, const InitialUncertainty &initial);

        void reset() override;
        void predict(double dt) override;
        void updateWheelSpeed(double speed, double variance) override;
        void updateKinematicYawRate(double yaw_rate, double variance) override;
        void updateGyro(double gyro_z, double variance) override;
        void updateGyroBias(double gyro_z, double variance) override;
        Estimate estimate() const override;

    private:
        void scalarUpdate(double measurement, double predicted, const Eigen::RowVectorXd &H, double variance);

        NoiseDensities noise_;
        InitialUncertainty initial_;
        ExtendedKalmanFilter filter_;
    };

    class ParticleEstimator : public StateEstimator
    {
    public:
        ParticleEstimator(size_t particle_count, const NoiseDensities &noise, const InitialUncertainty &initial, unsigned int seed = 0);

        void reset() override;
        void predict(double dt) override;
        void updateWheelSpeed(double speed, double variance) override;
        void updateKinematicYawRate(double yaw_rate, double variance) override;
        void updateGyro(double gyro_z, double variance) override;
        void updateGyroBias(double gyro_z, double variance) override;
        Estimate estimate() const override;
        std::vector<StateVector> particles() const override;

    private:
        void ensembleUpdate(double measurement, double variance, const std::function<double(const Eigen::VectorXd &)> &predicted_measurement);

        size_t particle_count_;
        NoiseDensities noise_;
        InitialUncertainty initial_;
        ParticleFilter filter_;
    };
}
