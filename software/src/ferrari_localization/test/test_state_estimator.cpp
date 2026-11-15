#include <cmath>
#include <memory>
#include <random>

#include <gtest/gtest.h>

#include "ferrari_localization/state_estimator.hpp"

using namespace ferrari_localization;

namespace
{
    struct Scenario
    {
        double speed = 1.0;
        double yaw_rate = 0.5;
        double gyro_bias = 0.05;
        double gyro_std = 0.01;
        double wheel_speed_std = 0.05;
        double kinematic_yaw_rate_std = 0.2;
        double duration = 20.0;
        double dt = 0.02;
    };

    struct Truth
    {
        double x;
        double y;
        double yaw;
    };

    Truth runScenario(StateEstimator &estimator, const Scenario &s, unsigned int seed)
    {
        std::mt19937 rng(seed);
        std::normal_distribution<double> normal(0.0, 1.0);

        Truth truth{0.0, 0.0, 0.0};
        const int steps = static_cast<int>(s.duration / s.dt);
        for (int i = 0; i < steps; ++i)
        {
            const double yaw_mid = truth.yaw + 0.5 * s.yaw_rate * s.dt;
            truth.x += s.speed * std::cos(yaw_mid) * s.dt;
            truth.y += s.speed * std::sin(yaw_mid) * s.dt;
            truth.yaw += s.yaw_rate * s.dt;

            const double gyro = s.yaw_rate + s.gyro_bias + s.gyro_std * normal(rng);
            estimator.predict(gyro, s.dt);
            estimator.updateWheelSpeed(s.speed + s.wheel_speed_std * normal(rng), s.wheel_speed_std * s.wheel_speed_std);
            estimator.updateYawRate(s.yaw_rate + s.kinematic_yaw_rate_std * normal(rng), s.kinematic_yaw_rate_std * s.kinematic_yaw_rate_std);
        }
        return truth;
    }

    double positionError(const Estimate &e, const Truth &t)
    {
        return std::hypot(e.state(X) - t.x, e.state(Y) - t.y);
    }

    double yawError(const Estimate &e, const Truth &t)
    {
        return std::abs(std::remainder(e.state(YAW) - t.yaw, 2.0 * M_PI));
    }
}

TEST(ExtendedKalmanFilter, ScalarUpdateMatchesAnalyticSolution)
{
    ExtendedKalmanFilter filter;
    filter.reset(Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Identity(1, 1));

    Eigen::VectorXd z(1), h(1);
    z << 1.0;
    h << 0.0;
    filter.update(z, h, Eigen::MatrixXd::Identity(1, 1), Eigen::MatrixXd::Identity(1, 1));

    EXPECT_NEAR(filter.state()(0), 0.5, 1e-12);
    EXPECT_NEAR(filter.covariance()(0, 0), 0.5, 1e-12);
}

TEST(ExtendedKalmanFilter, PredictGrowsCovarianceByProcessNoise)
{
    ExtendedKalmanFilter filter;
    filter.reset(Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Identity(1, 1));

    filter.predict(Eigen::VectorXd::Zero(1), 2.0 * Eigen::MatrixXd::Identity(1, 1), 0.5 * Eigen::MatrixXd::Identity(1, 1));

    EXPECT_NEAR(filter.covariance()(0, 0), 4.5, 1e-12);
}

TEST(MotionModel, JacobianMatchesFiniteDifferences)
{
    StateVector x;
    x << 1.0, -2.0, 0.7, 1.3, 0.04;
    const double gyro = 0.4;
    const double dt = 0.05;

    StateMatrix analytic;
    propagateMotion(x, gyro, dt, &analytic);

    StateMatrix numeric;
    const double eps = 1e-6;
    for (int j = 0; j < STATE_DIM; ++j)
    {
        StateVector plus = x, minus = x;
        plus(j) += eps;
        minus(j) -= eps;
        numeric.col(j) = (propagateMotion(plus, gyro, dt) - propagateMotion(minus, gyro, dt)) / (2.0 * eps);
    }

    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(MotionModel, StraightLineMotion)
{
    StateVector x = StateVector::Zero();
    x(SPEED) = 2.0;

    for (int i = 0; i < 50; ++i)
    {
        x = propagateMotion(x, 0.0, 0.02);
    }

    EXPECT_NEAR(x(X), 2.0, 1e-9);
    EXPECT_NEAR(x(Y), 0.0, 1e-9);
    EXPECT_NEAR(x(YAW), 0.0, 1e-9);
}

TEST(EkfEstimator, EstimatesGyroBiasAtStandstill)
{
    EkfEstimator estimator(NoiseDensities{}, InitialUncertainty{});
    std::mt19937 rng(1);
    std::normal_distribution<double> normal(0.0, 0.01);

    for (int i = 0; i < 250; ++i)
    {
        const double gyro = 0.05 + normal(rng);
        estimator.predict(gyro, 0.02);
        estimator.updateGyroBias(gyro, 1e-4);
    }

    const Estimate e = estimator.estimate();
    EXPECT_NEAR(e.state(GYRO_BIAS), 0.05, 0.005);
    EXPECT_NEAR(e.yaw_rate, 0.0, 0.02);
    EXPECT_LT(e.covariance(GYRO_BIAS, GYRO_BIAS), 1e-4);
}

TEST(EkfEstimator, WheelSpeedUpdateReducesSpeedUncertainty)
{
    EkfEstimator estimator(NoiseDensities{}, InitialUncertainty{});
    estimator.predict(0.0, 0.02);
    const double before = estimator.estimate().covariance(SPEED, SPEED);

    estimator.updateWheelSpeed(1.0, 0.0025);

    EXPECT_LT(estimator.estimate().covariance(SPEED, SPEED), before);
    EXPECT_GT(estimator.estimate().state(SPEED), 0.5);
}

TEST(EkfEstimator, TracksCircleWithBiasedGyro)
{
    EkfEstimator estimator(NoiseDensities{}, InitialUncertainty{});
    const Scenario scenario;
    const Truth truth = runScenario(estimator, scenario, 7);
    const Estimate e = estimator.estimate();

    EXPECT_LT(positionError(e, truth), 0.5);
    EXPECT_LT(yawError(e, truth), 0.1);
    EXPECT_NEAR(e.state(GYRO_BIAS), scenario.gyro_bias, 0.02);
    EXPECT_NEAR(e.state(SPEED), scenario.speed, 0.1);
}

TEST(ParticleEstimator, TracksCircleWithBiasedGyro)
{
    ParticleEstimator estimator(500, NoiseDensities{}, InitialUncertainty{}, 3);
    const Scenario scenario;
    const Truth truth = runScenario(estimator, scenario, 7);
    const Estimate e = estimator.estimate();

    EXPECT_LT(positionError(e, truth), 1.0);
    EXPECT_LT(yawError(e, truth), 0.2);
    EXPECT_NEAR(e.state(GYRO_BIAS), scenario.gyro_bias, 0.03);
    EXPECT_NEAR(e.state(SPEED), scenario.speed, 0.15);
}

TEST(ParticleFilter, ResamplingRestoresUniformWeights)
{
    ParticleFilter filter(5);
    filter.initialize(200, [](std::mt19937 &rng)
                      {
        std::normal_distribution<double> normal(0.0, 1.0);
        Eigen::VectorXd p(1);
        p << normal(rng);
        return p; });

    filter.reweight([](const Eigen::VectorXd &p)
                    { return std::exp(-0.5 * (p(0) - 1.0) * (p(0) - 1.0) / 0.01); });
    EXPECT_LT(filter.effectiveSampleSize(), 100.0);

    EXPECT_TRUE(filter.resampleIfNeeded());
    EXPECT_NEAR(filter.effectiveSampleSize(), 200.0, 1e-9);
    EXPECT_NEAR(filter.mean()(0), 1.0, 0.1);
}

TEST(ParticleFilter, ZeroLikelihoodFallsBackToUniformWeights)
{
    ParticleFilter filter(1);
    filter.initialize(10, [](std::mt19937 &)
                      { return Eigen::VectorXd::Zero(1); });

    filter.reweight([](const Eigen::VectorXd &)
                    { return 0.0; });

    for (double w : filter.weights())
    {
        EXPECT_NEAR(w, 0.1, 1e-12);
    }
}

TEST(ParticleFilter, MeanOfAnglesHandlesWrapAround)
{
    ParticleFilter filter(1);
    int counter = 0;
    filter.initialize(2, [&counter](std::mt19937 &)
                      {
        Eigen::VectorXd p(1);
        p << (counter++ == 0 ? M_PI - 0.1 : -M_PI + 0.1);
        return p; });

    EXPECT_NEAR(std::abs(filter.mean({0})(0)), M_PI, 1e-9);
}
