#include <algorithm>
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "ferrari_sensing/exposure_controller.hpp"

using namespace ferrari_sensing;

namespace
{
    struct Frame
    {
        double luma;
        double saturated;
    };

    Frame capture(const ExposureSetting &setting, double radiance, const ExposureParams &params)
    {
        const double relative = setting.exposure_us * setting.gain / (params.initial_exposure_us * params.initial_gain);
        const double linear = relative * radiance;
        return {255.0 * std::pow(std::clamp(linear, 0.0, 1.0), 1.0 / params.gamma), linear >= 1.0 ? 1.0 : 0.0};
    }

    double run(ExposureController &controller, double radiance, int frames, const ExposureParams &params, double noise_std = 0.0, unsigned int seed = 1)
    {
        std::mt19937 rng(seed);
        std::normal_distribution<double> noise(0.0, noise_std > 0.0 ? noise_std : 1.0);
        Frame frame{};
        for (int i = 0; i < frames; ++i)
        {
            frame = capture(controller.current(), radiance, params);
            const double luma = noise_std > 0.0 ? frame.luma + noise(rng) : frame.luma;
            controller.update(luma, frame.saturated);
        }
        return capture(controller.current(), radiance, params).luma;
    }
}

TEST(ExposureController, ConvergesToTargetAcrossSceneBrightness)
{
    const ExposureParams params;
    for (double radiance : {0.04, 0.1, 0.2, 0.6, 3.0, 6.0})
    {
        ExposureController controller(params);
        const double luma = run(controller, radiance, 80, params);
        EXPECT_NEAR(luma, params.target_luma, 10.0) << "radiance " << radiance;
    }
}

TEST(ExposureController, RecoversQuicklyFromWhiteOut)
{
    const ExposureParams params;
    ExposureController controller(params);
    run(controller, 0.2, 60, params);

    int frames_to_unsaturate = 0;
    for (; frames_to_unsaturate < 20; ++frames_to_unsaturate)
    {
        const Frame frame = capture(controller.current(), 4.0, params);
        if (frame.saturated == 0.0 && frame.luma < 200.0)
        {
            break;
        }
        controller.update(frame.luma, frame.saturated);
    }
    EXPECT_LE(frames_to_unsaturate, 3);

    EXPECT_NEAR(run(controller, 4.0, 12, params), params.target_luma, 12.0);
}

TEST(ExposureController, FollowsSceneGettingDarker)
{
    const ExposureParams params;
    ExposureController controller(params);
    run(controller, 0.6, 60, params);

    EXPECT_NEAR(run(controller, 0.03, 45, params), params.target_luma, 12.0);
}

TEST(ExposureController, StaysStableOnStaticScene)
{
    const ExposureParams params;
    ExposureController controller(params);
    run(controller, 0.2, 80, params);

    const ExposureSetting settled = controller.current();
    run(controller, 0.2, 60, params);

    EXPECT_DOUBLE_EQ(controller.current().exposure_us, settled.exposure_us);
    EXPECT_DOUBLE_EQ(controller.current().gain, settled.gain);
}

TEST(ExposureController, DoesNotRandomWalkOnNoisyMeasurements)
{
    const ExposureParams params;
    ExposureController controller(params);
    run(controller, 0.2, 80, params);

    double min_total = 1e18, max_total = 0.0;
    std::mt19937 rng(9);
    std::normal_distribution<double> noise(0.0, 4.0);
    for (int i = 0; i < 200; ++i)
    {
        const Frame frame = capture(controller.current(), 0.2, params);
        controller.update(frame.luma + noise(rng), frame.saturated);
        const double total = controller.current().exposure_us * controller.current().gain;
        min_total = std::min(min_total, total);
        max_total = std::max(max_total, total);
    }
    EXPECT_LT(max_total / min_total, 1.4);
}

TEST(ExposureController, RespectsLimitsAndPrefersLowGain)
{
    const ExposureParams params;

    ExposureController dim(params);
    run(dim, 0.0001, 100, params);
    EXPECT_DOUBLE_EQ(dim.current().exposure_us, params.max_exposure_us);
    EXPECT_DOUBLE_EQ(dim.current().gain, params.max_gain);

    ExposureController bright(params);
    run(bright, 1000.0, 100, params);
    EXPECT_GE(bright.current().exposure_us, params.min_exposure_us);
    EXPECT_DOUBLE_EQ(bright.current().gain, params.min_gain);

    ExposureController moderate(params);
    run(moderate, 0.5, 80, params);
    EXPECT_DOUBLE_EQ(moderate.current().gain, params.min_gain);
    EXPECT_LE(moderate.current().exposure_us, params.max_exposure_us);
}
