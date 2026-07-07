#include <cv_bridge/cv_bridge.hpp>
#include "ferrari_sensing/camera.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

CameraNode::CameraNode() : Node("camera_node")
{
    bool enable_manual_exposure = this->declare_parameter<bool>("enable_manual_exposure", true);
    int exposure_time = this->declare_parameter<int>("exposure_time", 15000);
    double analogue_gain = this->declare_parameter<double>("analogue_gain", 2.0);

    publish_rate_ = this->declare_parameter<double>("publish_rate", 30.0);
    jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 80);

    camera_info_.width = this->declare_parameter<int>("image_width", 640);
    camera_info_.height = this->declare_parameter<int>("image_height", 480);
    camera_info_.header.frame_id = this->declare_parameter<std::string>("camera_name", "camera");
    camera_info_.distortion_model = this->declare_parameter<std::string>("distortion_model", "plumb_bob");
    camera_info_.d = this->declare_parameter<std::vector<double>>("distortion_coefficients", std::vector<double>(5, 0.0));
    auto k_vec = this->declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>(9, 0.0));
    auto r_vec = this->declare_parameter<std::vector<double>>("rectification_matrix", std::vector<double>(9, 0.0));
    auto p_vec = this->declare_parameter<std::vector<double>>("projection_matrix", std::vector<double>(12, 0.0));
    std::copy(k_vec.begin(), k_vec.end(), camera_info_.k.begin());
    std::copy(r_vec.begin(), r_vec.end(), camera_info_.r.begin());
    std::copy(p_vec.begin(), p_vec.end(), camera_info_.p.begin());

    auto qos = rclcpp::QoS(5).best_effort();
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", qos);
    image_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);
    image_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_compressed", qos);

    // GStreamer pipeline for onboard camera
    std::string pipeline = "libcamerasrc ";

    if (enable_manual_exposure)
    {
        pipeline += "exposure-time-mode=1 analogue-gain-mode=1 ";
        pipeline += "exposure-time=" + std::to_string(exposure_time) + " ";
        pipeline += "analogue-gain=" + std::to_string(analogue_gain) + " ! ";
    }
    else
    {
        pipeline += "exposure-time-mode=0 analogue-gain-mode=0 ! ";
    }

    pipeline += "video/x-raw, width=" + std::to_string(camera_info_.width) + ", height=" + std::to_string(camera_info_.height) + ", framerate=" + std::to_string(publish_rate_) + "/1 ! ";
    pipeline += "videoconvert ! video/x-raw, format=BGR ! ";
    pipeline += "appsink drop=true";

    // Open camera
    cap_.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera with GStreamer pipeline");
        throw std::runtime_error("Cannot open camera");
    }

    RCLCPP_INFO(this->get_logger(), "Camera opened: %dx%d @ %.1f FPS",
                static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)),
                cap_.get(cv::CAP_PROP_FPS));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CameraNode::cameraAcquisitionLoop, this));
}

CameraNode::~CameraNode()
{
    if (cap_.isOpened())
    {
        cap_.release();
    }
    RCLCPP_INFO(this->get_logger(), "Camera released");
}

void CameraNode::cameraAcquisitionLoop()
{
    bool return_code;
    cv::Mat frame;
    std::vector<uchar> jpeg_buffer;
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

    // Capture frame from camera
    return_code = cap_.read(frame);
    if (!return_code || frame.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
        return;
    }

    // Encode frame to JPEG
    return_code = cv::imencode(".jpg", frame, jpeg_buffer, encode_params);
    if (!return_code)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to encode frame to JPEG");
        return;
    }

    // Publish messages
    auto header = std_msgs::msg::Header();
    header.stamp = this->now();
    header.frame_id = "camera";

    auto camera_info_msg = sensor_msgs::msg::CameraInfo(camera_info_);
    camera_info_msg.header = header;

    auto image_raw_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    auto image_compressed_msg = sensor_msgs::msg::CompressedImage();
    image_compressed_msg.header = header;
    image_compressed_msg.format = "jpeg";
    image_compressed_msg.data = jpeg_buffer;

    camera_info_pub_->publish(camera_info_msg);
    image_raw_pub_->publish(*image_raw_msg);
    image_compressed_pub_->publish(image_compressed_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node failed: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
