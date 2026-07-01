#include <cv_bridge/cv_bridge.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include "ferrari_perception/line_detector.hpp"

using namespace std::chrono_literals;

LineDetectorNode::LineDetectorNode() : Node("line_detector")
{
    // Declare parameters
    this->declare_parameter("roi_top", 300);
    this->declare_parameter("canny_threshold_low", 50);
    this->declare_parameter("canny_threshold_high", 150);
    this->declare_parameter("hough_threshold", 30);
    this->declare_parameter("line_min_length", 50.0);
    this->declare_parameter("line_max_gap", 30.0);
    this->declare_parameter("enable_debug_image", true);
    this->declare_parameter("kalman_process_noise_q", 0.5);
    this->declare_parameter("kalman_measurement_noise_r", 5.0);

    // Get parameters
    roi_top_ = this->get_parameter("roi_top").as_int();
    canny_threshold_low_ = this->get_parameter("canny_threshold_low").as_int();
    canny_threshold_high_ = this->get_parameter("canny_threshold_high").as_int();
    hough_threshold_ = this->get_parameter("hough_threshold").as_int();
    line_min_length_ = this->get_parameter("line_min_length").as_double();
    line_max_gap_ = this->get_parameter("line_max_gap").as_double();
    enable_debug_image_ = this->get_parameter("enable_debug_image").as_bool();
    kalman_process_noise_q_ = this->get_parameter("kalman_process_noise_q").as_double();
    kalman_measurement_noise_r_ = this->get_parameter("kalman_measurement_noise_r").as_double();

    frame_width_ = 640;
    frame_height_ = 480;

    kalman_state_ = cv::Mat::zeros(4, 1, CV_32F);
    kalman_covariance_ = cv::Mat::eye(4, 4, CV_32F) * 1.0f;

    float dt = 0.033f;
    kalman_transition_ = cv::Mat::eye(4, 4, CV_32F);
    kalman_transition_.at<float>(0, 1) = dt;
    kalman_transition_.at<float>(2, 3) = dt;

    kalman_measurement_matrix_ = cv::Mat::zeros(2, 4, CV_32F);
    kalman_measurement_matrix_.at<float>(0, 0) = 1.0f;
    kalman_measurement_matrix_.at<float>(1, 2) = 1.0f;

    rclcpp::QoS qos_profile(2);
    qos_profile.best_effort();

    // Subscribing to standard uncompressed rectified image
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_rect",
        qos_profile,
        std::bind(&LineDetectorNode::image_callback, this, std::placeholders::_1));

    qos_profile.reliable();

    // Publishers using standard messages
    line_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/perception/line_pose", qos_profile);

    line_confidence_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/perception/line_confidence", qos_profile);

    if (enable_debug_image_)
    {
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/perception/line_debug", qos_profile);
    }

    RCLCPP_INFO(this->get_logger(), "LineDetectorNode initialized (Standard Messages)");
    last_log_time_ = this->now().seconds();
}

void LineDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        if (frame.empty())
            return;

        frame_width_ = frame.cols;
        frame_height_ = frame.rows;

        cv::Mat gray = preprocess_image(frame);
        cv::Mat edges = detect_edges(gray);
        std::vector<cv::Vec4f> lines = detect_lines(edges);

        float offset = 0.0f, angle = 0.0f, confidence = 0.0f;
        bool line_detected = extract_line_parameters(lines, offset, angle, confidence);

        if (line_detected)
        {
            update_kalman_filter(offset, angle);
        }
        predict_kalman_filter();

        float filtered_offset = kalman_state_.at<float>(0, 0);
        float filtered_angle = kalman_state_.at<float>(2, 0);

        // ===== Publish Standard Messages =====
        if (line_detected)
        {
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = msg->header.stamp;
            pose_msg.header.frame_id = msg->header.frame_id;

            // Map offset to Y (horizontal)
            pose_msg.pose.position.x = 0.0;
            pose_msg.pose.position.y = filtered_offset;
            pose_msg.pose.position.z = 0.0;

            // Map angle to Yaw using TF2
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, filtered_angle);
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            line_pose_pub_->publish(pose_msg);

            auto conf_msg = std_msgs::msg::Float32();
            conf_msg.data = confidence;
            line_confidence_pub_->publish(conf_msg);
        }

        if (enable_debug_image_ && !lines.empty())
        {
            cv::Vec4f best_line = fit_best_line(lines, frame_width_);
            cv::Mat debug_img = draw_debug_image(frame, best_line, filtered_offset, filtered_angle, confidence);

            auto debug_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, debug_img).toImageMsg();
            debug_pub_->publish(*debug_msg);
        }

        frame_count_++;
        double now = this->now().seconds();
        if (now - last_log_time_ > 5.0)
        {
            double fps = frame_count_ / (now - last_log_time_);
            RCLCPP_INFO(this->get_logger(),
                        "Detection: %.1f FPS | offset: %.1f px | angle: %.3f rad | detected: %s",
                        fps, filtered_offset, filtered_angle, line_detected ? "YES" : "NO");
            frame_count_ = 0;
            last_log_time_ = now;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in image callback: %s", e.what());
    }
}
cv::Mat LineDetectorNode::preprocess_image(const cv::Mat &frame)
{
    // Crop to bottom ROI (region closest to car, ~20cm)
    cv::Rect roi(0, roi_top_, frame.cols, frame.rows - roi_top_);
    cv::Mat cropped = frame(roi);

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.0);

    // Optional: Increase contrast with CLAHE (Contrast Limited Adaptive Histogram Equalization)
    // This helps with varying lighting conditions
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    cv::Mat enhanced;
    clahe->apply(blurred, enhanced);

    return enhanced;
}

cv::Mat LineDetectorNode::detect_edges(const cv::Mat &gray)
{
    // Canny edge detection
    cv::Mat edges;
    cv::Canny(gray, edges, canny_threshold_low_, canny_threshold_high_);

    // Dilate to connect nearby edges
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::dilate(edges, edges, kernel, cv::Point(-1, -1), 1);

    return edges;
}

std::vector<cv::Vec4f> LineDetectorNode::detect_lines(const cv::Mat &edges)
{
    std::vector<cv::Vec4f> lines;

    // Probabilistic Hough Transform
    // Parameters: rho resolution (1 px), theta resolution (1 deg), min votes, min length, max gap
    cv::HoughLinesP(edges,
                    lines,
                    1,                // rho: 1 pixel
                    CV_PI / 180.0,    // theta: 1 degree
                    hough_threshold_, // Votes needed
                    line_min_length_, // Minimum line length
                    line_max_gap_);   // Maximum gap between segments

    return lines;
}

bool LineDetectorNode::extract_line_parameters(const std::vector<cv::Vec4f> &lines,
                                               float &offset, float &angle, float &confidence)
{
    if (lines.empty())
    {
        offset = 0.0f;
        angle = 0.0f;
        confidence = 0.0f;
        return false;
    }

    // Find the best line (closest to vertical, in center of image)
    cv::Vec4f best_line = fit_best_line(lines, frame_width_);

    // Calculate offset from center
    offset = calculate_line_offset(best_line, frame_width_);

    // Calculate angle relative to vertical
    angle = calculate_line_angle(best_line);

    // Confidence based on:
    // 1. How vertical is the line (closer to vertical = more confidence)
    // 2. How centered is the line (near image center = more confidence)
    float angle_confidence = 1.0f - std::abs(angle) / (CV_PI / 2.0f);                          // 1.0 if vertical, 0 if horizontal
    float offset_confidence = std::max(0.0f, 1.0f - std::abs(offset) / (frame_width_ / 2.0f)); // 1.0 if centered
    float num_lines_confidence = std::min(1.0f, lines.size() / 5.0f);                          // More lines = more confidence

    confidence = (angle_confidence * 0.5f) + (offset_confidence * 0.3f) + (num_lines_confidence * 0.2f);

    return true;
}

float LineDetectorNode::calculate_line_offset(const cv::Vec4f &line, int image_width)
{
    // Line endpoints
    float x1 = line[0], y1 = line[1];
    float x2 = line[2], y2 = line[3];

    // Mid-point of line segment
    float mid_x = (x1 + x2) / 2.0f;

    // Offset from image center
    float offset = mid_x - (image_width / 2.0f);

    return offset;
}

float LineDetectorNode::calculate_line_angle(const cv::Vec4f &line)
{
    // Line endpoints
    float x1 = line[0], y1 = line[1];
    float x2 = line[2], y2 = line[3];

    // Direction vector
    float dx = x2 - x1;
    float dy = y2 - y1;

    // Angle relative to vertical (up is 0)
    // atan2(dx, -dy): measure rotation from vertical axis
    float angle = std::atan2(dx, -dy);

    // Clamp to [-π/2, π/2]
    if (angle > CV_PI / 2.0f)
        angle -= CV_PI;
    if (angle < -CV_PI / 2.0f)
        angle += CV_PI;

    return angle;
}

cv::Vec4f LineDetectorNode::fit_best_line(const std::vector<cv::Vec4f> &lines, int image_width)
{
    if (lines.empty())
    {
        return cv::Vec4f(image_width / 2.0f, 0, image_width / 2.0f, 100);
    }

    // Scoring criteria:
    // 1. How close to vertical (prefer angle near 0)
    // 2. How centered (prefer offset near 0)
    // 3. How long (prefer longer lines)

    cv::Vec4f best_line = lines[0];
    float best_score = -1e6f;

    for (const auto &line : lines)
    {
        float offset = calculate_line_offset(line, image_width);
        float angle = calculate_line_angle(line);
        float length = std::sqrt(std::pow(line[2] - line[0], 2) + std::pow(line[3] - line[1], 2));

        // Scoring: prefer vertical, centered, and long lines
        float angle_score = 10.0f * (1.0f - std::abs(angle) / (CV_PI / 4.0f));        // -1 if 45°, -10 if 90°
        float offset_score = 5.0f * (1.0f - std::abs(offset) / (image_width / 2.0f)); // -5 if at edge
        float length_score = 0.1f * length;                                           // Slight preference for longer lines

        float score = angle_score + offset_score + length_score;

        if (score > best_score)
        {
            best_score = score;
            best_line = line;
        }
    }

    return best_line;
}

void LineDetectorNode::update_kalman_filter(float measured_offset, float measured_angle)
{
    // Measurement vector z = [offset, angle]
    cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F);
    measurement.at<float>(0, 0) = measured_offset;
    measurement.at<float>(1, 0) = measured_angle;

    // Measurement covariance
    cv::Mat measurement_cov = cv::Mat::eye(2, 2, CV_32F) * kalman_measurement_noise_r_;

    // Kalman update (simplified):
    // 1. Predict: x_pred = A * x
    cv::Mat x_pred = kalman_transition_ * kalman_state_;
    cv::Mat P_pred = kalman_transition_ * kalman_covariance_ * kalman_transition_.t();
    P_pred += cv::Mat::eye(4, 4, CV_32F) * kalman_process_noise_q_;

    // 2. Update: Kalman gain K = P_pred * H^T / (H * P_pred * H^T + R)
    cv::Mat innovation = measurement - kalman_measurement_matrix_ * x_pred;
    cv::Mat S = kalman_measurement_matrix_ * P_pred * kalman_measurement_matrix_.t() + measurement_cov;
    cv::Mat K = P_pred * kalman_measurement_matrix_.t() * S.inv();

    // 3. State update: x = x_pred + K * innovation
    kalman_state_ = x_pred + K * innovation;
    kalman_covariance_ = (cv::Mat::eye(4, 4, CV_32F) - K * kalman_measurement_matrix_) * P_pred;
}

void LineDetectorNode::predict_kalman_filter()
{
    // Simple prediction step (for when no measurement available)
    // x_pred = A * x
    kalman_state_ = kalman_transition_ * kalman_state_;
    kalman_covariance_ = kalman_transition_ * kalman_covariance_ * kalman_transition_.t();
    kalman_covariance_ += cv::Mat::eye(4, 4, CV_32F) * kalman_process_noise_q_;
}

cv::Mat LineDetectorNode::draw_debug_image(const cv::Mat &frame,
                                           const cv::Vec4f &line,
                                           float offset, float angle, float confidence)
{
    cv::Mat debug = frame.clone();

    // Draw ROI boundary
    cv::line(debug, cv::Point(0, roi_top_), cv::Point(frame.cols, roi_top_),
             cv::Scalar(255, 0, 0), 2); // Blue line

    // Draw center line (green)
    cv::line(debug, cv::Point(frame.cols / 2, roi_top_), cv::Point(frame.cols / 2, frame.rows),
             cv::Scalar(0, 255, 0), 1);

    // Draw detected line (red)
    cv::line(debug,
             cv::Point(static_cast<int>(line[0]), roi_top_ + static_cast<int>(line[1])),
             cv::Point(static_cast<int>(line[2]), roi_top_ + static_cast<int>(line[3])),
             cv::Scalar(0, 0, 255), 3);

    // Draw text information
    char text[256];
    snprintf(text, sizeof(text), "Offset: %.1f px | Angle: %.3f rad", offset, angle);
    cv::putText(debug, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    snprintf(text, sizeof(text), "Confidence: %.2f", confidence);
    cv::putText(debug, text, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    return debug;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<LineDetectorNode>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node failed: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}