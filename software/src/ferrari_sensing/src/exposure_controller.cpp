#include "ferrari_sensing/exposure_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ferrari_sensing
{
    ExposureController::ExposureController(const ExposureParams &params) : params_(params)
    {
        reset();
    }

    void ExposureController::reset()
    {
        exposure_us_ = params_.initial_exposure_us;
        gain_ = params_.initial_gain;
    }

    ExposureSetting ExposureController::update(double mean_luma, double saturated_fraction)
    {
        const double measured = std::pow(std::clamp(mean_luma, 1.0, 254.0) / 255.0, params_.gamma);
        const double target = std::pow(params_.target_luma / 255.0, params_.gamma);
        double error_ev = std::log2(target / measured);

        if (saturated_fraction > params_.saturation_threshold)
        {
            error_ev = std::min(error_ev, 0.0) - params_.saturation_penalty_ev;
        }

        if (std::abs(error_ev) < params_.deadband_ev)
        {
            return current();
        }

        const double step_ev = error_ev > 0.0
                                   ? std::min(params_.brighten_rate * error_ev, params_.max_brighten_step_ev)
                                   : std::max(params_.darken_rate * error_ev, -params_.max_darken_step_ev);

        const double min_total = params_.min_exposure_us * params_.min_gain;
        const double max_total = params_.max_exposure_us * params_.max_gain;
        const double total = std::clamp(exposure_us_ * gain_ * std::exp2(step_ev), min_total, max_total);

        if (total <= params_.max_exposure_us * params_.min_gain)
        {
            gain_ = params_.min_gain;
            exposure_us_ = std::max(total / gain_, params_.min_exposure_us);
        }
        else
        {
            exposure_us_ = params_.max_exposure_us;
            gain_ = std::clamp(total / exposure_us_, params_.min_gain, params_.max_gain);
        }
        return current();
    }
}
