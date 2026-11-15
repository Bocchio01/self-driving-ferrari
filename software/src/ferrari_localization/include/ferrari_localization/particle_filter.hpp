#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <random>
#include <vector>

#include <Eigen/Dense>

namespace ferrari_localization
{
    class ParticleFilter
    {
    public:
        using State = Eigen::VectorXd;
        using Sampler = std::function<State(std::mt19937 &)>;
        using Propagator = std::function<void(State &, std::mt19937 &)>;
        using Likelihood = std::function<double(const State &)>;

        explicit ParticleFilter(unsigned int seed = 0) : rng_(seed) {}

        void initialize(size_t count, const Sampler &sampler)
        {
            particles_.clear();
            particles_.reserve(count);
            for (size_t i = 0; i < count; ++i)
            {
                particles_.push_back(sampler(rng_));
            }
            weights_.assign(count, 1.0 / static_cast<double>(count));
        }

        void predict(const Propagator &propagate)
        {
            for (auto &particle : particles_)
            {
                propagate(particle, rng_);
            }
        }

        void reweight(const Likelihood &likelihood)
        {
            double total = 0.0;
            for (size_t i = 0; i < particles_.size(); ++i)
            {
                weights_[i] *= likelihood(particles_[i]);
                total += weights_[i];
            }

            if (!(total > 1e-300))
            {
                std::fill(weights_.begin(), weights_.end(), 1.0 / static_cast<double>(weights_.size()));
                return;
            }

            for (auto &w : weights_)
            {
                w /= total;
            }
        }

        double effectiveSampleSize() const
        {
            double sum_squares = 0.0;
            for (double w : weights_)
            {
                sum_squares += w * w;
            }
            return sum_squares > 0.0 ? 1.0 / sum_squares : 0.0;
        }

        bool resampleIfNeeded(double threshold_ratio = 0.5)
        {
            const size_t n = particles_.size();
            if (effectiveSampleSize() >= threshold_ratio * static_cast<double>(n))
            {
                return false;
            }

            std::vector<State> resampled;
            resampled.reserve(n);

            std::uniform_real_distribution<double> uniform(0.0, 1.0 / static_cast<double>(n));
            const double offset = uniform(rng_);
            double cumulative = weights_[0];
            size_t index = 0;

            for (size_t i = 0; i < n; ++i)
            {
                const double target = offset + static_cast<double>(i) / static_cast<double>(n);
                while (target > cumulative && index + 1 < n)
                {
                    ++index;
                    cumulative += weights_[index];
                }
                resampled.push_back(particles_[index]);
            }

            particles_ = std::move(resampled);
            std::fill(weights_.begin(), weights_.end(), 1.0 / static_cast<double>(n));
            return true;
        }

        State mean(const std::vector<int> &angle_indices = {}) const
        {
            State result = State::Zero(particles_.front().size());
            for (size_t i = 0; i < particles_.size(); ++i)
            {
                result += weights_[i] * particles_[i];
            }

            for (int index : angle_indices)
            {
                double sin_sum = 0.0;
                double cos_sum = 0.0;
                for (size_t i = 0; i < particles_.size(); ++i)
                {
                    sin_sum += weights_[i] * std::sin(particles_[i](index));
                    cos_sum += weights_[i] * std::cos(particles_[i](index));
                }
                result(index) = std::atan2(sin_sum, cos_sum);
            }
            return result;
        }

        Eigen::MatrixXd covariance(const State &mean_state, const std::vector<int> &angle_indices = {}) const
        {
            const int dim = static_cast<int>(mean_state.size());
            Eigen::MatrixXd result = Eigen::MatrixXd::Zero(dim, dim);
            for (size_t i = 0; i < particles_.size(); ++i)
            {
                State residual = particles_[i] - mean_state;
                for (int index : angle_indices)
                {
                    residual(index) = std::remainder(residual(index), 2.0 * M_PI);
                }
                result += weights_[i] * residual * residual.transpose();
            }
            return result;
        }

        std::vector<State> &particles() { return particles_; }
        const std::vector<State> &particles() const { return particles_; }
        const std::vector<double> &weights() const { return weights_; }
        std::mt19937 &rng() { return rng_; }

    private:
        std::vector<State> particles_;
        std::vector<double> weights_;
        std::mt19937 rng_;
    };
}
