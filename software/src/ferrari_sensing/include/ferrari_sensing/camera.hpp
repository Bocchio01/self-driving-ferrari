#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    ~CameraNode();

private:
    void cameraAcquisitionLoop();
    void setCameraInfoCallback(const sensor_msgs::srv::SetCameraInfo::Request::SharedPtr req,
                               sensor_msgs::srv::SetCameraInfo::Response::SharedPtr res);

    // Node state
    cv::VideoCapture cap_;
    int publish_rate_;
    int jpeg_quality_;

    sensor_msgs::msg::CameraInfo camera_info_;

    // ROS objects
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};