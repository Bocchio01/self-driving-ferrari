#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    ~CameraNode();

private:
    void capture_and_publish();
    void publish_camera_info(const std_msgs::msg::Header &header);

    // Node state
    cv::VideoCapture cap_;
    std::string frame_id_;
    int jpeg_quality_;
    int width_;
    int height_;

    // Diagnostics
    int frame_count_;
    double last_log_time_;

    // ROS objects
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};