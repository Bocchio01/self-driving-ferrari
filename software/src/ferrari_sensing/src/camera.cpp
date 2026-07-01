#include <cv_bridge/cv_bridge.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_path.hpp>
#include "ferrari_sensing/camera.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

CameraNode::CameraNode()
    : Node("camera_publisher")
{
    publish_rate_ = this->declare_parameter<double>("publish_rate", 30.0);
    jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 80);

    std::string camera_info_file = this->declare_parameter<std::string>("camera_info_file", "config/camera_info.yaml");
    std::string camera_info_path = ament_index_cpp::get_package_share_path("ferrari_sensing") / camera_info_file;

    if (!this->loadCameraInfo(camera_info_path))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load camera info from YAML file");
        throw std::runtime_error("Failed to load camera info from YAML file");
    }

    auto qos = rclcpp::QoS(2).best_effort();
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", qos);
    image_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);
    image_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_compressed", qos);

#ifdef USE_CAMERA_INFO_SERVICE
    set_camera_info_srv_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
        "camera/set_camera_info",
        std::bind(&CameraNode::setCameraInfoCallback, this, _1, _2));
#endif

    // Open camera
    cap_.open("libcamerasrc ! video/x-raw, width=640, height=480 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true", cv::CAP_GSTREAMER);
    if (!cap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera with GStreamer pipeline");
        throw std::runtime_error("Cannot open camera");
    }

    // Set camera properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, publish_rate_);

    // Get actual resolution
    width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    RCLCPP_INFO(this->get_logger(), "Camera opened: %dx%d @ %.1f FPS", width_, height_, actual_fps);

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

    auto image_raw_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    auto image_compressed_msg = sensor_msgs::msg::CompressedImage();
    image_compressed_msg.header = header;
    image_compressed_msg.format = "jpeg";
    image_compressed_msg.data = jpeg_buffer;

    auto camera_info_msg = sensor_msgs::msg::CameraInfo(camera_info_);
    camera_info_msg.header = header;

    image_raw_pub_->publish(*image_raw_msg);
    image_compressed_pub_->publish(image_compressed_msg);
    camera_info_pub_->publish(camera_info_msg);
}

void CameraNode::setCameraInfoCallback(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
{
    try
    {
        camera_info_ = request->camera_info;

        RCLCPP_INFO(this->get_logger(), "Successfully updated camera calibration parameters!");

        response->success = true;
        response->status_message = "Camera info updated successfully.";
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->status_message = std::string("Failed to update camera info: ") + e.what();
    }
}

bool CameraNode::loadCameraInfo(const std::string &file_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(file_path);

        camera_info_.header.frame_id = config["camera_name"].as<std::string>();
        camera_info_.height = config["image_height"].as<int>();
        camera_info_.width = config["image_width"].as<int>();

        camera_info_.distortion_model = config["distortion_model"].as<std::string>();
        auto d_node = config["distortion_coefficients"]["data"];
        for (const auto &val : d_node)
        {
            camera_info_.d.push_back(val.as<double>());
        }

        auto k_node = config["camera_matrix"]["data"];
        for (size_t i = 0; i < 9; ++i)
        {
            camera_info_.k[i] = k_node[i].as<double>();
        }

        auto r_node = config["rectification_matrix"]["data"];
        for (size_t i = 0; i < 9; ++i)
        {
            camera_info_.r[i] = r_node[i].as<double>();
        }

        auto p_node = config["projection_matrix"]["data"];
        for (size_t i = 0; i < 12; ++i)
        {
            camera_info_.p[i] = p_node[i].as<double>();
        }

        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error parsing YAML: %s", e.what());
        return false;
    }
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