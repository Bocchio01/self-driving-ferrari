#include "ferrari_perception/line_detector.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>

using std::placeholders::_1;

LineDetectorNode::LineDetectorNode(const rclcpp::NodeOptions &options) : Node("line_detector", options)
{
    std::string transport_hint_ = this->declare_parameter<std::string>("image_transport", "raw");

    src_points_norm_ = this->declare_parameter<std::vector<double>>(
        "warp_src_points_norm",
        {0.35, 0.55,   // top-left
         0.65, 0.55,   // top-right
         1.00, 1.00,   // bottom-right
         0.00, 1.00}); // bottom-left

    dst_points_norm_ = this->declare_parameter<std::vector<double>>(
        "warp_dst_points_norm",
        {0.20, 0.0,   // top-left
         0.80, 0.0,   // top-right
         0.80, 1.0,   // bottom-right
         0.20, 1.0}); // bottom-left

    const int warped_width = declare_parameter<int>("warped_width", 300);
    const int warped_height = declare_parameter<int>("warped_height", 400);
    warped_size_ = cv::Size(warped_width, warped_height);

    num_windows_ = declare_parameter<int>("num_windows", 10);

    hsv_threshold_ = this->declare_parameter<std::vector<int64_t>>(
        "hsv_threshold",
        {0, 179,     // H min/max
         0, 255,     // S min/max
         200, 255}); // V min/max

    meters_per_pixel_y_ = declare_parameter<double>("meters_per_pixel_y", 0.002);
    meters_per_pixel_x_ = declare_parameter<double>("meters_per_pixel_x", 0.002);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("line_path", 10);

    if (publish_debug_image_)
    {
        debug_image_pub_ = image_transport::create_publisher(
            *this,
            "debug_image",
            rclcpp::SensorDataQoS());
    }

    image_sub_ = image_transport::create_subscription(
        *this,
        "image_rect",
        std::bind(&LineDetectorNode::imgCallback, this, _1),
        transport_hint_,
        rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "line_detector started");
}

void LineDetectorNode::imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    const cv::Mat frame = imgToCvMat(msg);
    if (frame.empty())
    {
        return;
    }

    const cv::Mat warped = warpPerspective(frame);
    const cv::Mat blurred = applyGaussianBlur(warped);
    const cv::Mat binary = colorThreshold(blurred);
    const std::vector<cv::Point2f> line_points = findLineCenter(binary, 0, binary.rows);

    if (!line_points.empty())
    {
        nav_msgs::msg::Path path_msg;

        path_msg.header.stamp = msg->header.stamp;
        path_msg.header.frame_id = msg->header.frame_id;

        const double warped_h = binary.rows;
        const double warped_w = binary.cols;

        for (const auto &pt : line_points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            // pose.pose.position.x = pt.x;
            // pose.pose.position.y = pt.y;
            pose.pose.position.x = (warped_h - pt.y) * meters_per_pixel_y_;        // forward distance
            pose.pose.position.y = -(pt.x - warped_w / 2.0) * meters_per_pixel_x_; // lateral offset, left+
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0; // No rotation
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "No line detected in current frame");
    }

    if (publish_debug_image_)
    {
        const cv::Mat overlay = drawDebugOverlay(frame, binary, line_points);
        std_msgs::msg::Header debug_header = msg->header;
        auto debug_msg = cv_bridge::CvImage(debug_header, "bgr8", overlay).toImageMsg();
        debug_image_pub_.publish(debug_msg);
    }
}

cv::Mat LineDetectorNode::imgToCvMat(const sensor_msgs::msg::Image::ConstSharedPtr &msg) const
{
    try
    {
        return cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
        return cv::Mat();
    }
}

cv::Mat LineDetectorNode::applyGaussianBlur(const cv::Mat &frame) const
{
    cv::Mat blurred;
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
    return blurred;
}

cv::Mat LineDetectorNode::warpPerspective(const cv::Mat &frame) const
{
    if (!matrices_ready_)
    {
        const auto toPoints = [](const std::vector<double> &normalized, const cv::Size &size)
        {
            std::vector<cv::Point2f> points;
            points.reserve(4);

            for (size_t i = 0; i < 4; ++i)
            {
                points.emplace_back(
                    static_cast<float>(normalized[2 * i] * size.width),
                    static_cast<float>(normalized[2 * i + 1] * size.height));
            }

            return points;
        };

        const auto src = toPoints(src_points_norm_, frame.size());
        const auto dst = toPoints(dst_points_norm_, warped_size_);

        warp_matrix_ = cv::getPerspectiveTransform(src, dst);
        matrices_ready_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Computed perspective warp for frame size %dx%d -> warped %dx%d",
            frame.cols,
            frame.rows,
            warped_size_.width,
            warped_size_.height);
    }

    cv::Mat warped;
    cv::warpPerspective(frame, warped, warp_matrix_, warped_size_);

    return warped;
}

cv::Mat LineDetectorNode::colorThreshold(const cv::Mat &frame) const
{
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat binary;
    cv::inRange(
        hsv,
        cv::Scalar(hsv_threshold_[0], hsv_threshold_[2], hsv_threshold_[4]),
        cv::Scalar(hsv_threshold_[1], hsv_threshold_[3], hsv_threshold_[5]),
        binary);

    // Clean up small speckles / fill small gaps in the tape line.
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 25));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

    return binary;
}

std::vector<cv::Point2f> LineDetectorNode::findLineCenter(const cv::Mat &frame, int y_low, int y_high) const
{
    std::vector<cv::Point2f> line_points;

    const int window_height = (y_high - y_low) / num_windows_;
    int current_y = y_high;

    for (int window = 0; window < num_windows_; ++window)
    {
        const int y_start = current_y - window_height;
        const int y_end = current_y;

        cv::Mat window_roi = frame(cv::Range(y_start, y_end), cv::Range::all());

        cv::Moments moments = cv::moments(window_roi, true);
        cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00 + y_start);
        line_points.push_back(center);

        current_y -= window_height;
    }

    return line_points;
}

cv::Mat LineDetectorNode::drawDebugOverlay(
    const cv::Mat &original_frame,
    const cv::Mat &binary_warped,
    const std::vector<cv::Point2f> &warped_line_points) const
{
    // 1. Start with the original image as the background
    cv::Mat overlay = original_frame.clone();

    // 2. Draw the ROI (Warped Area bounds) on the original image in RED
    std::vector<cv::Point> roi_points;
    for (size_t i = 0; i < 4; ++i)
    {
        roi_points.emplace_back(
            static_cast<int>(src_points_norm_[2 * i] * overlay.cols),
            static_cast<int>(src_points_norm_[2 * i + 1] * overlay.rows));
    }
    std::vector<std::vector<cv::Point>> pts = {roi_points};
    cv::polylines(overlay, pts, true, cv::Scalar(0, 0, 255), 2); // Red trapezoid

    // 3. Unwarp the binary image back to the original perspective
    // We compute the inverse of the matrix you generated in warpPerspective()
    cv::Mat inv_warp_matrix = warp_matrix_.inv();
    cv::Mat unwarped_binary;
    cv::warpPerspective(binary_warped, unwarped_binary, inv_warp_matrix, overlay.size());

    // 4. Overlay the binary mask (Make the detected tape pixels BLUE)
    cv::Mat blue_mask(overlay.size(), overlay.type(), cv::Scalar(255, 0, 0));
    blue_mask.copyTo(overlay, unwarped_binary);

    // 5. Transform and draw the path points in GREEN
    if (!warped_line_points.empty())
    {
        std::vector<cv::Point2f> original_line_points;
        // Transform the 2D coordinates back to the original perspective
        cv::perspectiveTransform(warped_line_points, original_line_points, inv_warp_matrix);

        for (size_t i = 0; i < original_line_points.size(); ++i)
        {
            // Draw the center points
            cv::circle(overlay, original_line_points[i], 5, cv::Scalar(0, 255, 0), -1);

            // Connect the points with a line to visualize the steering path
            if (i > 0)
            {
                cv::line(overlay, original_line_points[i - 1], original_line_points[i], cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    return overlay;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}