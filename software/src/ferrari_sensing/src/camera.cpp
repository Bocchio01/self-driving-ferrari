#include "ferrari_sensing/camera.hpp"

using namespace std::chrono_literals;

CameraNode::CameraNode() : Node("camera_publisher"), frame_count_(0)
{
    // Declare parameters
    this->declare_parameter("gst_pipeline",
                            "libcamerasrc ! video/x-raw, width=640, height=480 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true");
    this->declare_parameter("frame_id", "camera");
    this->declare_parameter("publish_rate", 30.0);
    this->declare_parameter("jpeg_quality", 80);

    // Get parameters
    std::string gst_pipeline = this->get_parameter("gst_pipeline").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

    // Setup QoS for video streaming (best effort, low latency, depth 2)
    rclcpp::QoS qos_profile(2);
    qos_profile.best_effort();
    qos_profile.keep_last(2);

    // Publishers
    image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_compressed", qos_profile);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", qos_profile);

    // Open camera
    cap_.open(gst_pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera with GStreamer pipeline");
        throw std::runtime_error("Cannot open camera");
    }

    // Set camera properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, publish_rate);

    // Get actual resolution
    width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    RCLCPP_INFO(this->get_logger(), "Camera opened: %dx%d @ %.1f FPS", width_, height_, actual_fps);

    last_log_time_ = this->now().seconds();

    // Timer for publishing
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(timer_period, std::bind(&CameraNode::capture_and_publish, this));
}

CameraNode::~CameraNode()
{
    if (cap_.isOpened())
    {
        cap_.release();
    }
    RCLCPP_INFO(this->get_logger(), "Camera released");
}

void CameraNode::capture_and_publish()
{
    cv::Mat frame;
    bool ret = cap_.read(frame);

    if (!ret || frame.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
        return;
    }

    // Encode to JPEG
    std::vector<uchar> jpeg_buffer;
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    bool ret_encode = cv::imencode(".jpg", frame, jpeg_buffer, encode_params);

    if (!ret_encode)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to encode frame to JPEG");
        return;
    }

    // Create and populate the compressed image message
    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data = jpeg_buffer;

    // Publish
    image_pub_->publish(msg);
    publish_camera_info(msg.header);

    // Diagnostic logging every 5 seconds
    frame_count_++;
    double now = this->now().seconds();
    if (now - last_log_time_ > 5.0)
    {
        double fps = frame_count_ / (now - last_log_time_);
        double msg_size_kb = jpeg_buffer.size() / 1024.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: %.1f FPS, frame size: %.1f KB", fps, msg_size_kb);

        frame_count_ = 0;
        last_log_time_ = now;
    }
}

void CameraNode::publish_camera_info(const std_msgs::msg::Header &header)
{
    auto msg = sensor_msgs::msg::CameraInfo();
    msg.header = header;
    msg.height = height_;
    msg.width = width_;

    // IMX219 focal length (approximate)
    double focal_length_x = 525.0;
    double focal_length_y = 525.0;

    msg.k = {
        focal_length_x, 0.0, width_ / 2.0,
        0.0, focal_length_y, height_ / 2.0,
        0.0, 0.0, 1.0};

    msg.p = {
        focal_length_x, 0.0, width_ / 2.0, 0.0,
        0.0, focal_length_y, height_ / 2.0, 0.0,
        0.0, 0.0, 1.0, 0.0};

    msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    msg.distortion_model = "plumb_bob";
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    msg.binning_x = 1;
    msg.binning_y = 1;

    camera_info_pub_->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<CameraNode>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node failed: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}