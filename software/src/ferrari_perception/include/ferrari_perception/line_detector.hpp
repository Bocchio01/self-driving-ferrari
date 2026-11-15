#pragma once

#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ferrari_perception/line_detection.hpp"
#include "ferrari_perception/msg/line_state.hpp"

class LineDetectorNode : public rclcpp::Node
{
public:
    explicit LineDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void publishPath(const std_msgs::msg::Header &header, const std::vector<ferrari_perception::PathSample> &samples);
    void publishState(const std_msgs::msg::Header &header, const ferrari_perception::LineDetection &detection);
    cv::Mat drawDebugOverlay(const cv::Mat &frame, const ferrari_perception::LineDetection &detection,
                             const std::vector<ferrari_perception::PathSample> &samples) const;

    ferrari_perception::LineDetectorParams detector_params_;
    std::unique_ptr<ferrari_perception::LineDetector> detector_;
    std::unique_ptr<ferrari_perception::LineTracker> tracker_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_image_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<ferrari_perception::msg::LineState>::SharedPtr state_pub_;

    std::string base_frame_;
    double camera_x_offset_ = 0.0;
    double path_step_ = 0.05;
    double state_reference_x_ = 0.1;
    bool publish_debug_image_ = true;
};
