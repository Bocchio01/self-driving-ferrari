#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace ferrari_perception
{
    struct LineDetectorParams
    {
        std::array<double, 8> warp_src_norm{0.2296875, 0.575, 0.76875, 0.575, 0.9765625, 0.7229166666666667, 0.0234375, 0.7125};
        std::array<double, 8> warp_dst_norm{0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0};
        int warped_width = 330;
        int warped_height = 210;
        double meters_per_pixel_x = 0.001015;
        double meters_per_pixel_y = 0.0010105;

        double background_sigma_px = 25.0;
        double contrast_gain = 2.0;
        double min_threshold = 12.0;
        double max_line_width_m = 0.07;
        int min_component_area_px = 400;
        double min_extent_ratio = 0.3;
        int band_height_px = 6;
        int min_band_pixels = 6;
        int robust_iterations = 2;
        double outlier_floor_m = 0.005;
        int min_points = 8;
        double min_coverage = 0.25;
        double ridge_margin = 6.0;
        double ridge_relative_margin = 0.1;
    };

    struct LineDetection
    {
        bool valid = false;
        Eigen::Vector3d coefficients = Eigen::Vector3d::Zero();
        double x_min = 0.0;
        double x_max = 0.0;
        double rms_error = 0.0;
        double coverage = 0.0;
        int point_count = 0;
        double mean_luma = 0.0;
        double saturated_fraction = 0.0;
        double dark_fraction = 0.0;
        std::vector<cv::Point2f> warped_points;
        cv::Mat warped;
        cv::Mat mask;

        double lateralOffsetAt(double x) const { return coefficients(0) + coefficients(1) * x + coefficients(2) * x * x; }
        double headingAt(double x) const { return std::atan(coefficients(1) + 2.0 * coefficients(2) * x); }
    };

    class LineDetector
    {
    public:
        explicit LineDetector(const LineDetectorParams &params = LineDetectorParams());

        LineDetection detect(const cv::Mat &bgr_frame);

        const cv::Mat &warpMatrix() const { return warp_matrix_; }
        const LineDetectorParams &params() const { return params_; }

    private:
        cv::Mat warp(const cv::Mat &frame);
        cv::Mat contrastMask(const cv::Mat &value) const;
        cv::Mat selectLineComponent(const cv::Mat &mask) const;
        bool isRidge(const cv::Mat &value, int top, int bottom, int start, int end) const;
        std::vector<cv::Point2f> bandCenters(const cv::Mat &mask, const cv::Mat &value) const;
        bool fitPolynomial(const std::vector<cv::Point2f> &points, LineDetection &detection) const;

        LineDetectorParams params_;
        cv::Mat warp_matrix_;
        cv::Size warp_frame_size_;
    };

    struct TrackerParams
    {
        double process_c0 = 2.5e-5;
        double process_c1 = 5.0e-4;
        double process_c2 = 5.0e-2;
        double measurement_c0 = 0.01;
        double measurement_c1 = 0.08;
        double measurement_c2 = 1.0;
        double gate_c0 = 0.08;
        int max_lost_frames = 10;
        int reacquire_frames = 4;
    };

    struct PathSample
    {
        double x;
        double y;
        double yaw;
    };

    class LineTracker
    {
    public:
        explicit LineTracker(const TrackerParams &params = TrackerParams());

        void reset();
        bool update(const LineDetection &detection);

        bool valid() const { return valid_; }
        int lostFrames() const { return lost_frames_; }
        const Eigen::Vector3d &coefficients() const { return coefficients_; }
        double lateralOffsetAt(double x) const { return coefficients_(0) + coefficients_(1) * x + coefficients_(2) * x * x; }
        double headingAt(double x) const { return std::atan(coefficients_(1) + 2.0 * coefficients_(2) * x); }
        std::vector<PathSample> sample(double step) const;

    private:
        void initialize(const LineDetection &detection);

        TrackerParams params_;
        bool valid_ = false;
        bool initialized_ = false;
        int lost_frames_ = 0;
        int rejected_frames_ = 0;
        double x_min_ = 0.0;
        double x_max_ = 0.0;
        Eigen::Vector3d coefficients_ = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Identity();
    };
}
