#include "ferrari_localization/state_estimator.hpp"

#include <cmath>

namespace ferrari_localization
{
    StateVector propagateMotion(const StateVector &x, double dt, StateMatrix *jacobian)
    {
        const double yaw_mid = x(YAW) + 0.5 * x(YAW_RATE) * dt;
        const double sin_mid = std::sin(yaw_mid);
        const double cos_mid = std::cos(yaw_mid);

        StateVector next = x;
        next(X) += x(SPEED) * cos_mid * dt;
        next(Y) += x(SPEED) * sin_mid * dt;
        next(YAW) = std::remainder(x(YAW) + x(YAW_RATE) * dt, 2.0 * M_PI);

        if (jacobian)
        {
            *jacobian = StateMatrix::Identity();
            (*jacobian)(X, YAW) = -x(SPEED) * sin_mid * dt;
            (*jacobian)(X, SPEED) = cos_mid * dt;
            (*jacobian)(X, YAW_RATE) = -0.5 * x(SPEED) * sin_mid * dt * dt;
            (*jacobian)(Y, YAW) = x(SPEED) * cos_mid * dt;
            (*jacobian)(Y, SPEED) = sin_mid * dt;
            (*jacobian)(Y, YAW_RATE) = 0.5 * x(SPEED) * cos_mid * dt * dt;
            (*jacobian)(YAW, YAW_RATE) = dt;
        }
        return next;
    }

    namespace
    {
        StateVector initialStd(const InitialUncertainty &initial)
        {
            StateVector s;
            s << initial.pose, initial.pose, initial.pose, initial.speed, initial.yaw_rate, initial.gyro_bias;
            return s;
        }
    }

    EkfEstimator::EkfEstimator(const NoiseDensities &noise, const InitialUncertainty &initial)
        : noise_(noise), initial_(initial)
    {
        reset();
    }

    void EkfEstimator::reset()
    {
        const StateVector s = initialStd(initial_);
        filter_.reset(Eigen::VectorXd::Zero(STATE_DIM), (s.array().square()).matrix().asDiagonal());
    }

    void EkfEstimator::predict(double dt)
    {
        StateMatrix F;
        const StateVector x_next = propagateMotion(filter_.state(), dt, &F);

        StateVector q;
        q << noise_.position * noise_.position, noise_.position * noise_.position, 0.0,
            noise_.speed * noise_.speed, noise_.yaw_rate * noise_.yaw_rate, noise_.gyro_bias * noise_.gyro_bias;
        const StateMatrix Q = (q * dt).asDiagonal();

        filter_.predict(x_next, F, Q);
    }

    void EkfEstimator::scalarUpdate(double measurement, double predicted, const Eigen::RowVectorXd &H, double variance)
    {
        Eigen::VectorXd z(1), h(1);
        z << measurement;
        h << predicted;
        filter_.update(z, h, H, Eigen::MatrixXd::Constant(1, 1, variance));
    }

    void EkfEstimator::updateWheelSpeed(double speed, double variance)
    {
        Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(STATE_DIM);
        H(SPEED) = 1.0;
        scalarUpdate(speed, filter_.state()(SPEED), H, variance);
    }

    void EkfEstimator::updateKinematicYawRate(double yaw_rate, double variance)
    {
        Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(STATE_DIM);
        H(YAW_RATE) = 1.0;
        scalarUpdate(yaw_rate, filter_.state()(YAW_RATE), H, variance);
    }

    void EkfEstimator::updateGyro(double gyro_z, double variance)
    {
        Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(STATE_DIM);
        H(YAW_RATE) = 1.0;
        H(GYRO_BIAS) = 1.0;
        scalarUpdate(gyro_z, filter_.state()(YAW_RATE) + filter_.state()(GYRO_BIAS), H, variance);
    }

    void EkfEstimator::updateGyroBias(double gyro_z, double variance)
    {
        Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(STATE_DIM);
        H(GYRO_BIAS) = 1.0;
        scalarUpdate(gyro_z, filter_.state()(GYRO_BIAS), H, variance);
    }

    Estimate EkfEstimator::estimate() const
    {
        Estimate e;
        e.state = filter_.state();
        e.covariance = filter_.covariance();
        return e;
    }

    ParticleEstimator::ParticleEstimator(size_t particle_count, const NoiseDensities &noise, const InitialUncertainty &initial, unsigned int seed)
        : particle_count_(particle_count), noise_(noise), initial_(initial), filter_(seed)
    {
        reset();
    }

    void ParticleEstimator::reset()
    {
        const StateVector s = initialStd(initial_);
        filter_.initialize(particle_count_, [s](std::mt19937 &rng)
                           {
            std::normal_distribution<double> normal(0.0, 1.0);
            Eigen::VectorXd particle(STATE_DIM);
            for (int i = 0; i < STATE_DIM; ++i)
            {
                particle(i) = s(i) * normal(rng);
            }
            return particle; });
    }

    void ParticleEstimator::predict(double dt)
    {
        const double sqrt_dt = std::sqrt(dt);
        const NoiseDensities noise = noise_;

        filter_.predict([dt, sqrt_dt, noise](Eigen::VectorXd &particle, std::mt19937 &rng)
                        {
            std::normal_distribution<double> normal(0.0, 1.0);
            StateVector next = propagateMotion(particle, dt);
            next(X) += noise.position * sqrt_dt * normal(rng);
            next(Y) += noise.position * sqrt_dt * normal(rng);
            next(SPEED) += noise.speed * sqrt_dt * normal(rng);
            next(YAW_RATE) += noise.yaw_rate * sqrt_dt * normal(rng);
            next(GYRO_BIAS) += noise.gyro_bias * sqrt_dt * normal(rng);
            particle = next; });
    }

    void ParticleEstimator::ensembleUpdate(double measurement, double variance, const std::function<double(const Eigen::VectorXd &)> &predicted_measurement)
    {
        auto &particles = filter_.particles();
        const auto &weights = filter_.weights();
        const size_t n = particles.size();

        std::vector<double> predicted(n);
        double predicted_mean = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            predicted[i] = predicted_measurement(particles[i]);
            predicted_mean += weights[i] * predicted[i];
        }

        const StateVector mean = filter_.mean({YAW});
        double predicted_variance = 0.0;
        StateVector cross_covariance = StateVector::Zero();
        for (size_t i = 0; i < n; ++i)
        {
            const double dh = predicted[i] - predicted_mean;
            StateVector dx = Eigen::Map<const StateVector>(particles[i].data()) - mean;
            dx(YAW) = std::remainder(dx(YAW), 2.0 * M_PI);
            predicted_variance += weights[i] * dh * dh;
            cross_covariance += weights[i] * dh * dx;
        }

        const StateVector gain = cross_covariance / (predicted_variance + variance);
        std::normal_distribution<double> perturbation(0.0, std::sqrt(variance));
        for (size_t i = 0; i < n; ++i)
        {
            Eigen::Map<StateVector> particle(particles[i].data());
            particle += gain * (measurement + perturbation(filter_.rng()) - predicted[i]);
            particle(YAW) = std::remainder(particle(YAW), 2.0 * M_PI);
        }
    }

    void ParticleEstimator::updateWheelSpeed(double speed, double variance)
    {
        ensembleUpdate(speed, variance, [](const Eigen::VectorXd &p)
                       { return p(SPEED); });
    }

    void ParticleEstimator::updateKinematicYawRate(double yaw_rate, double variance)
    {
        ensembleUpdate(yaw_rate, variance, [](const Eigen::VectorXd &p)
                       { return p(YAW_RATE); });
    }

    void ParticleEstimator::updateGyro(double gyro_z, double variance)
    {
        ensembleUpdate(gyro_z, variance, [](const Eigen::VectorXd &p)
                       { return p(YAW_RATE) + p(GYRO_BIAS); });
    }

    void ParticleEstimator::updateGyroBias(double gyro_z, double variance)
    {
        ensembleUpdate(gyro_z, variance, [](const Eigen::VectorXd &p)
                       { return p(GYRO_BIAS); });
    }

    Estimate ParticleEstimator::estimate() const
    {
        const std::vector<int> angles = {YAW};
        const Eigen::VectorXd mean = filter_.mean(angles);

        Estimate e;
        e.state = mean;
        e.covariance = filter_.covariance(mean, angles);
        return e;
    }
}
