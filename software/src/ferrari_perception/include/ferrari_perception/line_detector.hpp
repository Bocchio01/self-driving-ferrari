#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <deque>
#include <cmath>

class LineDetectorNode : public rclcpp::Node
{
public:
    LineDetectorNode();
    ~LineDetectorNode() = default;

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    cv::Mat preprocess_image(const cv::Mat &frame);
    cv::Mat detect_edges(const cv::Mat &gray);
    std::vector<cv::Vec4f> detect_lines(const cv::Mat &edges);
    bool extract_line_parameters(const std::vector<cv::Vec4f> &lines,
                                 float &offset, float &angle, float &confidence);

    float calculate_line_offset(const cv::Vec4f &line, int image_width);
    float calculate_line_angle(const cv::Vec4f &line);
    cv::Vec4f fit_best_line(const std::vector<cv::Vec4f> &lines, int image_width);

    void update_kalman_filter(float measured_offset, float measured_angle);
    void predict_kalman_filter();

    cv::Mat draw_debug_image(const cv::Mat &frame,
                             const cv::Vec4f &line,
                             float offset, float angle, float confidence);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Publishers using standard messages
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr line_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr line_confidence_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

    // Parameters
    int roi_top_;
    int canny_threshold_low_;
    int canny_threshold_high_;
    int hough_threshold_;
    double line_min_length_;
    double line_max_gap_;
    bool enable_debug_image_;

    // Kalman filter state
    cv::Mat kalman_state_;
    cv::Mat kalman_covariance_;
    cv::Mat kalman_transition_;
    cv::Mat kalman_measurement_matrix_;
    float kalman_process_noise_q_;
    float kalman_measurement_noise_r_;

    std::deque<float> offset_history_;
    std::deque<float> angle_history_;
    std::deque<float> confidence_history_;
    int history_size_ = 30;

    int frame_width_;
    int frame_height_;
    uint64_t frame_count_ = 0;
    double last_log_time_ = 0.0;
};