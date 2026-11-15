#include "ferrari_perception/line_detection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ferrari_perception
{
    LineDetector::LineDetector(const LineDetectorParams &params) : params_(params) {}

    cv::Mat LineDetector::warp(const cv::Mat &frame)
    {
        if (warp_matrix_.empty() || warp_frame_size_ != frame.size())
        {
            std::vector<cv::Point2f> src, dst;
            for (size_t i = 0; i < 4; ++i)
            {
                src.emplace_back(static_cast<float>(params_.warp_src_norm[2 * i] * frame.cols), static_cast<float>(params_.warp_src_norm[2 * i + 1] * frame.rows));
                dst.emplace_back(static_cast<float>(params_.warp_dst_norm[2 * i] * params_.warped_width), static_cast<float>(params_.warp_dst_norm[2 * i + 1] * params_.warped_height));
            }
            warp_matrix_ = cv::getPerspectiveTransform(src, dst);
            warp_frame_size_ = frame.size();
        }

        cv::Mat warped;
        cv::warpPerspective(frame, warped, warp_matrix_, cv::Size(params_.warped_width, params_.warped_height));
        return warped;
    }

    cv::Mat LineDetector::contrastMask(const cv::Mat &value) const
    {
        cv::Mat value_f, background, contrast, contrast_8u, mask;
        value.convertTo(value_f, CV_32F);
        cv::GaussianBlur(value_f, background, cv::Size(0, 0), params_.background_sigma_px);
        contrast = (value_f - background) * params_.contrast_gain;
        contrast.convertTo(contrast_8u, CV_8U);

        const double otsu = cv::threshold(contrast_8u, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(contrast_8u, mask, std::max(otsu, params_.min_threshold), 255, cv::THRESH_BINARY);

        const cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 11));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);
        return mask;
    }

    cv::Mat LineDetector::selectLineComponent(const cv::Mat &mask) const
    {
        cv::Mat labels, stats, centroids;
        const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

        const double max_width_px = params_.max_line_width_m / params_.meters_per_pixel_x;
        int best = -1;
        int best_area = 0;
        for (int i = 1; i < count; ++i)
        {
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);
            const int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            if (area < params_.min_component_area_px || height < params_.min_extent_ratio * mask.rows)
            {
                continue;
            }
            if (static_cast<double>(area) / height > max_width_px)
            {
                continue;
            }
            if (area > best_area)
            {
                best = i;
                best_area = area;
            }
        }

        if (best < 0)
        {
            return cv::Mat();
        }
        return labels == best;
    }

    bool LineDetector::isRidge(const cv::Mat &value, int top, int bottom, int start, int end) const
    {
        const int run_width = end - start + 1;
        const int flank = std::max(run_width, 6);
        const double run_mean = cv::mean(value(cv::Range(top, bottom), cv::Range(start, end + 1)))[0];
        const double margin = std::max(params_.ridge_margin, params_.ridge_relative_margin * run_mean);

        const int left_start = std::max(0, start - flank);
        const int right_end = std::min(value.cols, end + 1 + flank);
        if (left_start < start)
        {
            const double left_mean = cv::mean(value(cv::Range(top, bottom), cv::Range(left_start, start)))[0];
            if (run_mean - left_mean < margin)
            {
                return false;
            }
        }
        if (end + 1 < right_end)
        {
            const double right_mean = cv::mean(value(cv::Range(top, bottom), cv::Range(end + 1, right_end)))[0];
            if (run_mean - right_mean < margin)
            {
                return false;
            }
        }
        return true;
    }

    std::vector<cv::Point2f> LineDetector::bandCenters(const cv::Mat &mask, const cv::Mat &value) const
    {
        std::vector<cv::Point2f> centers;
        const double max_run_px = 2.0 * params_.max_line_width_m / params_.meters_per_pixel_x;

        for (int bottom = mask.rows; bottom - params_.band_height_px >= 0; bottom -= params_.band_height_px)
        {
            const int top = bottom - params_.band_height_px;
            cv::Mat column_sum;
            cv::reduce(mask.rowRange(top, bottom) / 255, column_sum, 0, cv::REDUCE_SUM, CV_32S);

            double best_weight = 0.0;
            double best_center = 0.0;
            int best_start = 0;
            int best_end = 0;
            int run_start = -1;
            double weight = 0.0;
            double moment = 0.0;

            for (int x = 0; x <= mask.cols; ++x)
            {
                const int c = x < mask.cols ? column_sum.at<int>(0, x) : 0;
                const bool on = c * 2 >= params_.band_height_px;
                if (on)
                {
                    if (run_start < 0)
                    {
                        run_start = x;
                        weight = 0.0;
                        moment = 0.0;
                    }
                    weight += c;
                    moment += static_cast<double>(c) * x;
                }
                else if (run_start >= 0)
                {
                    if (x - run_start <= max_run_px && weight > best_weight)
                    {
                        best_weight = weight;
                        best_center = moment / weight;
                        best_start = run_start;
                        best_end = x - 1;
                    }
                    run_start = -1;
                }
            }

            if (best_weight >= params_.min_band_pixels && isRidge(value, top, bottom, best_start, best_end))
            {
                centers.emplace_back(static_cast<float>(best_center + 0.5), 0.5f * static_cast<float>(top + bottom));
            }
        }
        return centers;
    }

    bool LineDetector::fitPolynomial(const std::vector<cv::Point2f> &points, LineDetection &detection) const
    {
        if (points.size() < 4)
        {
            return false;
        }

        const double width = params_.warped_width;
        const double height = params_.warped_height;
        std::vector<Eigen::Vector2d> metric;
        metric.reserve(points.size());
        for (const auto &p : points)
        {
            metric.emplace_back((height - p.y) * params_.meters_per_pixel_y, -(p.x - width / 2.0) * params_.meters_per_pixel_x);
        }

        std::vector<bool> inlier(metric.size(), true);
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (int iteration = 0; iteration <= params_.robust_iterations; ++iteration)
        {
            std::vector<size_t> used;
            for (size_t i = 0; i < metric.size(); ++i)
            {
                if (inlier[i])
                {
                    used.push_back(i);
                }
            }
            if (used.size() < 4)
            {
                return false;
            }

            Eigen::MatrixXd A(used.size(), 3);
            Eigen::VectorXd b(used.size());
            for (size_t k = 0; k < used.size(); ++k)
            {
                const double x = metric[used[k]](0);
                A.row(k) << 1.0, x, x * x;
                b(k) = metric[used[k]](1);
            }
            c = A.colPivHouseholderQr().solve(b);

            std::vector<double> residuals(metric.size());
            for (size_t i = 0; i < metric.size(); ++i)
            {
                const double x = metric[i](0);
                residuals[i] = std::abs(metric[i](1) - (c(0) + c(1) * x + c(2) * x * x));
            }
            std::vector<double> sorted = residuals;
            std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
            const double limit = std::max(3.0 * 1.4826 * sorted[sorted.size() / 2], params_.outlier_floor_m);
            for (size_t i = 0; i < metric.size(); ++i)
            {
                inlier[i] = residuals[i] <= limit;
            }
        }

        double squared = 0.0;
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::lowest();
        int count = 0;
        for (size_t i = 0; i < metric.size(); ++i)
        {
            if (!inlier[i])
            {
                continue;
            }
            const double x = metric[i](0);
            const double residual = metric[i](1) - (c(0) + c(1) * x + c(2) * x * x);
            squared += residual * residual;
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            ++count;
        }
        const double coverage = (x_max - x_min) / (height * params_.meters_per_pixel_y);
        if (count < params_.min_points || coverage < params_.min_coverage)
        {
            return false;
        }

        detection.coefficients = c;
        detection.x_min = x_min;
        detection.x_max = x_max;
        detection.rms_error = std::sqrt(squared / count);
        detection.point_count = count;
        detection.coverage = coverage;
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (inlier[i])
            {
                detection.warped_points.push_back(points[i]);
            }
        }
        return true;
    }

    LineDetection LineDetector::detect(const cv::Mat &bgr_frame)
    {
        LineDetection detection;
        if (bgr_frame.empty())
        {
            return detection;
        }

        detection.warped = warp(bgr_frame);

        cv::Mat hsv;
        cv::cvtColor(detection.warped, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);
        const cv::Mat &value = channels[2];

        detection.mean_luma = cv::mean(value)[0];
        detection.saturated_fraction = static_cast<double>(cv::countNonZero(value >= 250)) / value.total();
        detection.dark_fraction = static_cast<double>(cv::countNonZero(value <= 5)) / value.total();

        cv::Mat mask = selectLineComponent(contrastMask(value));
        if (mask.empty())
        {
            detection.mask = cv::Mat::zeros(value.size(), CV_8U);
            return detection;
        }
        detection.mask = mask;

        detection.valid = fitPolynomial(bandCenters(mask, value), detection);
        return detection;
    }

    LineTracker::LineTracker(const TrackerParams &params) : params_(params) {}

    void LineTracker::reset()
    {
        valid_ = false;
        initialized_ = false;
        lost_frames_ = 0;
        rejected_frames_ = 0;
    }

    void LineTracker::initialize(const LineDetection &detection)
    {
        coefficients_ = detection.coefficients;
        covariance_ = Eigen::Vector3d(params_.measurement_c0, params_.measurement_c1, params_.measurement_c2).array().square().matrix().asDiagonal();
        x_min_ = detection.x_min;
        x_max_ = detection.x_max;
        initialized_ = true;
        valid_ = true;
        lost_frames_ = 0;
        rejected_frames_ = 0;
    }

    bool LineTracker::update(const LineDetection &detection)
    {
        if (!detection.valid)
        {
            ++lost_frames_;
            if (lost_frames_ > params_.max_lost_frames)
            {
                valid_ = false;
            }
            return false;
        }

        if (!initialized_ || !valid_)
        {
            initialize(detection);
            return true;
        }

        if (std::abs(detection.coefficients(0) - coefficients_(0)) > params_.gate_c0)
        {
            ++rejected_frames_;
            ++lost_frames_;
            if (rejected_frames_ >= params_.reacquire_frames)
            {
                initialize(detection);
                return true;
            }
            if (lost_frames_ > params_.max_lost_frames)
            {
                valid_ = false;
            }
            return false;
        }

        const Eigen::Matrix3d process = Eigen::Vector3d(params_.process_c0, params_.process_c1, params_.process_c2).asDiagonal();
        const Eigen::Matrix3d measurement = Eigen::Vector3d(params_.measurement_c0, params_.measurement_c1, params_.measurement_c2).array().square().matrix().asDiagonal();

        covariance_ += process;
        const Eigen::Matrix3d gain = covariance_ * (covariance_ + measurement).inverse();
        coefficients_ += gain * (detection.coefficients - coefficients_);
        covariance_ = (Eigen::Matrix3d::Identity() - gain) * covariance_;

        const double blend = 0.5;
        x_min_ = (1.0 - blend) * x_min_ + blend * detection.x_min;
        x_max_ = (1.0 - blend) * x_max_ + blend * detection.x_max;
        lost_frames_ = 0;
        rejected_frames_ = 0;
        valid_ = true;
        return true;
    }

    std::vector<PathSample> LineTracker::sample(double step) const
    {
        std::vector<PathSample> samples;
        if (!valid_ || step <= 0.0)
        {
            return samples;
        }
        for (double x = x_min_; x <= x_max_ + 1e-9; x += step)
        {
            samples.push_back({x, lateralOffsetAt(x), headingAt(x)});
        }
        return samples;
    }
}
