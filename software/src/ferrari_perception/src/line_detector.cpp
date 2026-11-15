#include "ferrari_perception/line_detector.hpp"

#include <cmath>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace ferrari_perception;
using std::placeholders::_1;

namespace
{
    template <size_t N>
    std::array<double, N> toArray(const std::vector<double> &values, const std::array<double, N> &fallback)
    {
        if (values.size() != N)
        {
            return fallback;
        }
        std::array<double, N> result;
        std::copy(values.begin(), values.end(), result.begin());
        return result;
    }
}

LineDetectorNode::LineDetectorNode(const rclcpp::NodeOptions &options) : Node("line_detector", options)
{
    const std::string transport_hint = this->declare_parameter<std::string>("image_transport", "raw");

    LineDetectorParams p;
    p.warp_src_norm = toArray<8>(this->declare_parameter<std::vector<double>>("warp_src_points_norm", std::vector<double>(p.warp_src_norm.begin(), p.warp_src_norm.end())), p.warp_src_norm);
    p.warp_dst_norm = toArray<8>(this->declare_parameter<std::vector<double>>("warp_dst_points_norm", std::vector<double>(p.warp_dst_norm.begin(), p.warp_dst_norm.end())), p.warp_dst_norm);
    p.warped_width = this->declare_parameter<int>("warped_width", p.warped_width);
    p.warped_height = this->declare_parameter<int>("warped_height", p.warped_height);
    p.meters_per_pixel_x = this->declare_parameter<double>("meters_per_pixel_x", p.meters_per_pixel_x);
    p.meters_per_pixel_y = this->declare_parameter<double>("meters_per_pixel_y", p.meters_per_pixel_y);
    p.background_sigma_px = this->declare_parameter<double>("background_sigma_px", p.background_sigma_px);
    p.contrast_gain = this->declare_parameter<double>("contrast_gain", p.contrast_gain);
    p.min_threshold = this->declare_parameter<double>("min_threshold", p.min_threshold);
    p.max_line_width_m = this->declare_parameter<double>("max_line_width_m", p.max_line_width_m);
    p.min_points = this->declare_parameter<int>("min_points", p.min_points);
    p.min_coverage = this->declare_parameter<double>("min_coverage", p.min_coverage);
    detector_params_ = p;
    detector_ = std::make_unique<LineDetector>(p);

    TrackerParams t;
    t.measurement_c0 = this->declare_parameter<double>("tracker_measurement_c0", t.measurement_c0);
    t.process_c0 = this->declare_parameter<double>("tracker_process_c0", t.process_c0);
    t.gate_c0 = this->declare_parameter<double>("tracker_gate_c0", t.gate_c0);
    t.max_lost_frames = this->declare_parameter<int>("tracker_max_lost_frames", t.max_lost_frames);
    t.reacquire_frames = this->declare_parameter<int>("tracker_reacquire_frames", t.reacquire_frames);
    tracker_ = std::make_unique<LineTracker>(t);

    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    camera_x_offset_ = this->declare_parameter<double>("camera_x_offset", 0.0);
    path_step_ = this->declare_parameter<double>("path_step", 0.05);
    state_reference_x_ = this->declare_parameter<double>("state_reference_x", 0.1);
    publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("line_path", 10);
    state_pub_ = create_publisher<ferrari_perception::msg::LineState>("line_state", 10);
    if (publish_debug_image_)
    {
        debug_image_pub_ = image_transport::create_publisher(*this, "debug_image", rclcpp::SensorDataQoS());
    }

    image_sub_ = image_transport::create_subscription(
        *this, "image_rect", std::bind(&LineDetectorNode::imgCallback, this, _1), transport_hint, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "line_detector started");
}

void LineDetectorNode::imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    cv::Mat frame;
    try
    {
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
        return;
    }

    const LineDetection detection = detector_->detect(frame);
    tracker_->update(detection);

    std_msgs::msg::Header header = msg->header;
    header.frame_id = base_frame_;

    const auto samples = tracker_->sample(path_step_);
    if (tracker_->valid() && !samples.empty())
    {
        publishPath(header, samples);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No line detected");
    }
    publishState(header, detection);

    if (publish_debug_image_)
    {
        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", drawDebugOverlay(frame, detection, samples)).toImageMsg();
        debug_image_pub_.publish(debug_msg);
    }
}

void LineDetectorNode::publishPath(const std_msgs::msg::Header &header, const std::vector<PathSample> &samples)
{
    nav_msgs::msg::Path path;
    path.header = header;
    for (const auto &s : samples)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = camera_x_offset_ + s.x;
        pose.pose.position.y = s.y;
        pose.pose.orientation.z = std::sin(0.5 * s.yaw);
        pose.pose.orientation.w = std::cos(0.5 * s.yaw);
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
}

void LineDetectorNode::publishState(const std_msgs::msg::Header &header, const LineDetection &detection)
{
    ferrari_perception::msg::LineState state;
    state.header = header;
    state.detected = detection.valid;
    state.tracking = tracker_->valid();
    state.coverage = detection.coverage;
    state.rms_error = detection.rms_error;
    state.point_count = detection.point_count;
    state.lost_frames = tracker_->lostFrames();
    state.mean_luma = detection.mean_luma;
    state.saturated_fraction = detection.saturated_fraction;
    state.dark_fraction = detection.dark_fraction;

    if (tracker_->valid())
    {
        const double x = state_reference_x_;
        const Eigen::Vector3d &c = tracker_->coefficients();
        const double slope = c(1) + 2.0 * c(2) * x;
        state.lateral_offset = tracker_->lateralOffsetAt(x);
        state.heading = std::atan(slope);
        state.curvature = 2.0 * c(2) / std::pow(1.0 + slope * slope, 1.5);
    }
    else
    {
        state.lateral_offset = std::nan("");
        state.heading = std::nan("");
        state.curvature = std::nan("");
    }
    state_pub_->publish(state);
}

cv::Mat LineDetectorNode::drawDebugOverlay(const cv::Mat &frame, const LineDetection &detection, const std::vector<PathSample> &samples) const
{
    cv::Mat overlay = frame.clone();
    const auto &p = detector_params_;

    std::vector<cv::Point> roi;
    for (size_t i = 0; i < 4; ++i)
    {
        roi.emplace_back(static_cast<int>(p.warp_src_norm[2 * i] * overlay.cols), static_cast<int>(p.warp_src_norm[2 * i + 1] * overlay.rows));
    }
    cv::polylines(overlay, std::vector<std::vector<cv::Point>>{roi}, true, cv::Scalar(0, 0, 255), 2);

    const cv::Mat &warp_matrix = detector_->warpMatrix();
    if (warp_matrix.empty())
    {
        return overlay;
    }
    const cv::Mat inverse = warp_matrix.inv();

    if (!detection.mask.empty())
    {
        cv::Mat unwarped_mask;
        cv::warpPerspective(detection.mask, unwarped_mask, inverse, overlay.size());
        cv::Mat blue(overlay.size(), overlay.type(), cv::Scalar(255, 0, 0));
        blue.copyTo(overlay, unwarped_mask);
    }

    if (!samples.empty())
    {
        std::vector<cv::Point2f> warped;
        for (const auto &s : samples)
        {
            warped.emplace_back(static_cast<float>(p.warped_width / 2.0 - s.y / p.meters_per_pixel_x), static_cast<float>(p.warped_height - s.x / p.meters_per_pixel_y));
        }
        std::vector<cv::Point2f> original;
        cv::perspectiveTransform(warped, original, inverse);
        for (size_t i = 0; i < original.size(); ++i)
        {
            cv::circle(overlay, original[i], 4, cv::Scalar(0, 255, 0), -1);
            if (i > 0)
            {
                cv::line(overlay, original[i - 1], original[i], cv::Scalar(0, 255, 0), 2);
            }
        }
    }
    return overlay;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
