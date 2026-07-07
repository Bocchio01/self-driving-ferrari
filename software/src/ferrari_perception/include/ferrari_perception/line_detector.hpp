#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>

#include <optional>
#include <string>
#include <vector>

/**
 * @brief LineDetectorNode is a ROS2 node that detects a single tape/pen line on the floor from a low,
 * cockpit-mounted camera and publishes it as a nav_msgs/Path in the vehicle's base frame. The node performs
 * a series of image processing steps including Gaussian blur, inverse perspective mapping, HSV thresholding,
 * sliding-window search, polynomial fitting, and pixel-to-metric conversion to generate the path message.
 */
class LineDetectorNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the LineDetectorNode.
     * @param options Node options for initialization.
     */
    explicit LineDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    /**
     * @brief Callback function for processing incoming camera images.
     *
     * @details This function is called whenever a new image is received on the subscribed topic.
     * It performs the following steps:
     * 1. Converts the ROS image message to an OpenCV Mat.
     * 2. Applies a perspective warp to obtain a bird's-eye view.
     * 3. Applies Gaussian blur to reduce noise.
     * 4. Thresholds the image to isolate the line based on HSV values.
     * 5. Finds the center of the line using a sliding window search.
     * 6. Builds a nav_msgs/Path message from the detected line and publishes it.
     *
     * @param msg The incoming image message.
     */
    void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    /**
     * @brief Converts a ROS image message to an OpenCV Mat.
     * @param msg The incoming image message.
     * @return The converted OpenCV Mat.
     */
    cv::Mat imgToCvMat(const sensor_msgs::msg::Image::ConstSharedPtr &msg) const;

    /**
     * @brief Applies Gaussian blur to the input image.
     * @param frame The input image.
     * @return The blurred image.
     */
    cv::Mat applyGaussianBlur(const cv::Mat &frame) const;

    /**
     * @brief Applies a perspective warp to the input image to obtain a bird's-eye view.
     * @param frame The input image.
     * @return The warped image.
     */
    cv::Mat warpPerspective(const cv::Mat &frame) const;

    /**
     * @brief Thresholds the input image to isolate the line based on HSV values.
     * @param frame The input image.
     * @return The binary thresholded image.
     */
    cv::Mat colorThreshold(const cv::Mat &frame) const;

    /**
     * @brief Performs a sliding window search to find the center of the line in the warped image.
     * @param frame The binary warped image.
     * @param y_low The lower y-coordinate of the search window.
     * @param y_high The upper y-coordinate of the search window.
     * @return A sequence of points representing the detected line, or std::nullopt if no line is found.
     */
    std::vector<cv::Point2f> findLineCenter(const cv::Mat &frame, int y_low, int y_high) const;

    /**
     * @brief Draws a debug overlay on the input image showing the detected line and other relevant information.
     * @param original_frame The original input image.
     * @param binary_warped The binary warped image used for line detection.
     * @param warped_line_points The points representing the detected line in the warped image.
     * @return The image with the debug overlay.
     */
    cv::Mat drawDebugOverlay(const cv::Mat &original_frame,
                             const cv::Mat &binary_warped,
                             const std::vector<cv::Point2f> &warped_line_points) const;

    image_transport::Subscriber image_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    image_transport::Publisher debug_image_pub_;

    mutable cv::Mat warp_matrix_;
    cv::Size warped_size_;
    mutable bool matrices_ready_ = false;

    std::vector<double> src_points_norm_;
    std::vector<double> dst_points_norm_;
    std::vector<int64_t> hsv_threshold_;

    int num_windows_;

    bool publish_debug_image_ = true;
    double meters_per_pixel_y_, meters_per_pixel_x_;
};