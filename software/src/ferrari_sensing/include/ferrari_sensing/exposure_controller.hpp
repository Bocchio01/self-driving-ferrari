#pragma once

namespace ferrari_sensing
{
    struct ExposureParams
    {
        double target_luma = 115.0;
        double gamma = 2.2;
        double min_exposure_us = 200.0;
        double max_exposure_us = 8000.0;
        double min_gain = 1.0;
        double max_gain = 8.0;
        double initial_exposure_us = 5000.0;
        double initial_gain = 2.0;
        double deadband_ev = 0.15;
        double brighten_rate = 0.35;
        double darken_rate = 0.9;
        double max_brighten_step_ev = 0.4;
        double max_darken_step_ev = 3.0;
        double saturation_threshold = 0.02;
        double saturation_penalty_ev = 1.0;
    };

    struct ExposureSetting
    {
        double exposure_us;
        double gain;
    };

    class ExposureController
    {
    public:
        explicit ExposureController(const ExposureParams &params = ExposureParams());

        ExposureSetting update(double mean_luma, double saturated_fraction);
        ExposureSetting current() const { return {exposure_us_, gain_}; }
        void reset();

    private:
        ExposureParams params_;
        double exposure_us_;
        double gain_;
    };
}
