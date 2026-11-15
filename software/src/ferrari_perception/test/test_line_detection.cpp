#include <functional>
#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ferrari_perception/line_detection.hpp"

using namespace ferrari_perception;

namespace
{
    const std::vector<std::string> kImages = {
        "image.png", "image_analog_gain_2.png", "image_last.png", "image_last_last.png", "image_raw.png",
        "image_raw_tes.png", "image_rect.png", "image_rect_.png", "image_rect_paper.png", "image_rect_paper_bottom.png"};

    const std::vector<std::string> kSamePoseImages = {
        "image_analog_gain_2.png", "image_last.png", "image_last_last.png", "image_raw.png", "image_raw_tes.png",
        "image_rect.png", "image_rect_.png", "image_rect_paper.png", "image_rect_paper_bottom.png"};

    const std::vector<double> kProbeX = {0.04, 0.08, 0.12, 0.16};

    cv::Mat loadImage(const std::string &name)
    {
        cv::Mat image = cv::imread(std::string(TEST_IMAGE_DIR) + "/" + name);
        EXPECT_FALSE(image.empty()) << name;
        return image;
    }

    double maxSpread(const std::vector<LineDetection> &detections)
    {
        double spread = 0.0;
        for (double x : kProbeX)
        {
            double lo = std::numeric_limits<double>::max();
            double hi = std::numeric_limits<double>::lowest();
            for (const auto &d : detections)
            {
                lo = std::min(lo, d.lateralOffsetAt(x));
                hi = std::max(hi, d.lateralOffsetAt(x));
            }
            spread = std::max(spread, hi - lo);
        }
        return spread;
    }

    std::vector<LineDetection> detectAll(const std::function<cv::Mat(const cv::Mat &)> &transform)
    {
        LineDetector detector;
        std::vector<LineDetection> result;
        for (const auto &name : kImages)
        {
            result.push_back(detector.detect(transform(loadImage(name))));
        }
        return result;
    }
}

TEST(LineDetector, FindsLineInEveryCalibrationImage)
{
    const auto detections = detectAll([](const cv::Mat &m)
                                      { return m; });
    for (size_t i = 0; i < detections.size(); ++i)
    {
        EXPECT_TRUE(detections[i].valid) << kImages[i];
        EXPECT_GT(detections[i].coverage, 0.5) << kImages[i];
        EXPECT_LT(detections[i].rms_error, 0.01) << kImages[i];
    }
}

TEST(LineDetector, CenterlineAgreesAcrossExposuresAndWhiteBalance)
{
    LineDetector detector;
    std::vector<LineDetection> detections;
    for (const auto &name : kSamePoseImages)
    {
        detections.push_back(detector.detect(loadImage(name)));
    }
    EXPECT_LT(maxSpread(detections), 0.015);
}

TEST(LineDetector, RobustToHeavyJpegCompression)
{
    LineDetector detector;
    for (const auto &name : kImages)
    {
        const cv::Mat image = loadImage(name);
        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer, {cv::IMWRITE_JPEG_QUALITY, 30});
        const cv::Mat compressed = cv::imdecode(buffer, cv::IMREAD_COLOR);

        const LineDetection reference = detector.detect(image);
        const LineDetection degraded = detector.detect(compressed);

        ASSERT_TRUE(degraded.valid) << name;
        for (double x : kProbeX)
        {
            EXPECT_NEAR(degraded.lateralOffsetAt(x), reference.lateralOffsetAt(x), 0.01) << name;
        }
    }
}

TEST(LineDetector, NeverConfidentlyWrongUnderExposureChanges)
{
    LineDetector detector;
    for (double alpha : {0.35, 0.6, 1.0, 1.8})
    {
        int found = 0;
        for (const auto &name : kImages)
        {
            const cv::Mat image = loadImage(name);
            cv::Mat scaled;
            image.convertTo(scaled, -1, alpha, 0.0);

            const LineDetection reference = detector.detect(image);
            const LineDetection result = detector.detect(scaled);
            if (!result.valid)
            {
                continue;
            }
            ++found;
            for (double x = result.x_min + 0.01; x <= result.x_max - 0.01; x += 0.04)
            {
                EXPECT_NEAR(result.lateralOffsetAt(x), reference.lateralOffsetAt(x), 0.02) << name << " alpha " << alpha;
            }
        }
        if (alpha >= 0.6 && alpha <= 1.0)
        {
            EXPECT_EQ(found, static_cast<int>(kImages.size())) << "alpha " << alpha;
        }
        else
        {
            EXPECT_GE(found, 6) << "alpha " << alpha;
        }
    }
}

TEST(LineDetector, ReportsNoLineOnTexturedFloor)
{
    std::mt19937 rng(5);
    std::normal_distribution<double> noise(0.0, 18.0);
    cv::Mat image(480, 640, CV_8UC3);
    for (int r = 0; r < image.rows; ++r)
    {
        for (int c = 0; c < image.cols; ++c)
        {
            const uchar v = cv::saturate_cast<uchar>(70.0 + noise(rng));
            image.at<cv::Vec3b>(r, c) = cv::Vec3b(v, v, v);
        }
    }
    cv::GaussianBlur(image, image, cv::Size(0, 0), 1.5);

    LineDetector detector;
    EXPECT_FALSE(detector.detect(image).valid);
}

TEST(LineDetector, RejectsLargeWhiteSheets)
{
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(60, 60, 60));
    cv::rectangle(image, cv::Rect(40, 300, 220, 120), cv::Scalar(230, 230, 230), cv::FILLED);
    cv::rectangle(image, cv::Rect(420, 300, 200, 120), cv::Scalar(230, 230, 230), cv::FILLED);

    LineDetector detector;
    EXPECT_FALSE(detector.detect(image).valid);
}

TEST(LineDetector, ReportsLuminanceStatistics)
{
    LineDetector detector;
    const LineDetection dark = detector.detect(cv::Mat(480, 640, CV_8UC3, cv::Scalar(10, 10, 10)));
    const LineDetection white = detector.detect(cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255)));

    EXPECT_NEAR(dark.mean_luma, 10.0, 1.0);
    EXPECT_GT(white.saturated_fraction, 0.99);
    EXPECT_FALSE(dark.valid);
}

namespace
{
    LineDetection measurement(double c0, double c1 = 0.0, double c2 = 0.0)
    {
        LineDetection d;
        d.valid = true;
        d.coefficients << c0, c1, c2;
        d.x_min = 0.02;
        d.x_max = 0.18;
        return d;
    }
}

TEST(LineTracker, SmoothsMeasurementNoise)
{
    LineTracker tracker;
    std::mt19937 rng(2);
    std::normal_distribution<double> noise(0.0, 0.01);

    double raw_sq = 0.0, filtered_sq = 0.0;
    const int n = 300;
    for (int i = 0; i < n; ++i)
    {
        const double z = noise(rng);
        tracker.update(measurement(0.05 + z));
        if (i > 50)
        {
            raw_sq += z * z;
            filtered_sq += std::pow(tracker.coefficients()(0) - 0.05, 2);
        }
    }
    EXPECT_LT(filtered_sq, 0.6 * raw_sq);
}

TEST(LineTracker, RejectsIsolatedOutliersAndReacquiresOnPersistentJump)
{
    LineTracker tracker;
    for (int i = 0; i < 20; ++i)
    {
        tracker.update(measurement(0.0));
    }

    EXPECT_FALSE(tracker.update(measurement(0.3)));
    EXPECT_NEAR(tracker.coefficients()(0), 0.0, 0.01);

    bool accepted = false;
    for (int i = 0; i < 6 && !accepted; ++i)
    {
        accepted = tracker.update(measurement(0.3));
    }
    EXPECT_TRUE(accepted);
    EXPECT_NEAR(tracker.coefficients()(0), 0.3, 0.01);
}

TEST(LineTracker, HoldsThenInvalidatesWhenLineIsLost)
{
    TrackerParams params;
    params.max_lost_frames = 3;
    LineTracker tracker(params);
    tracker.update(measurement(0.02));

    for (int i = 0; i < 3; ++i)
    {
        tracker.update(LineDetection());
        EXPECT_TRUE(tracker.valid());
    }
    tracker.update(LineDetection());
    EXPECT_FALSE(tracker.valid());
}

TEST(LineTracker, SamplesPathWithTangentYaw)
{
    LineTracker tracker;
    tracker.update(measurement(0.01, 0.5, 0.0));

    const auto samples = tracker.sample(0.05);
    ASSERT_GE(samples.size(), 3u);
    EXPECT_NEAR(samples.front().yaw, std::atan(0.5), 1e-6);
    EXPECT_NEAR(samples.front().y, 0.01 + 0.5 * samples.front().x, 1e-6);
}
