#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ferrari_vehicle/msg/odom_lite.hpp"

struct Segment
{
    double x1, y1, x2, y2;
};

class LidarSimulatorNode : public rclcpp::Node
{
public:
    LidarSimulatorNode() : Node("lidar_simulator_node")
    {
        const auto hall = this->declare_parameter<std::vector<double>>("hall", {-6.0, 9.5, -15.0, 6.0});
        const auto boxes = this->declare_parameter<std::vector<double>>("boxes", {-4.5, -3.0, 1.0, 1.0, 8.0, -8.0, 1.2, 1.2, 1.0, -13.5, 1.5, 1.0, 7.0, 3.5, 1.0, 1.0, -4.0, -10.0, 1.0, 1.6});
        lidar_x_ = this->declare_parameter<double>("lidar_x", 0.11033);
        lidar_y_ = this->declare_parameter<double>("lidar_y", 0.0);
        lidar_yaw_ = this->declare_parameter<double>("lidar_yaw", 0.0);
        bins_ = this->declare_parameter<int>("scan_bins", 450);
        range_min_ = this->declare_parameter<double>("range_min", 0.02);
        range_max_ = this->declare_parameter<double>("range_max", 12.0);
        noise_std_ = this->declare_parameter<double>("range_noise_std", 0.01);
        frame_id_ = this->declare_parameter<std::string>("frame_id", "lidar_link");
        const double rate_hz = this->declare_parameter<double>("rate_hz", 10.0);

        addRectangle((hall[0] + hall[1]) / 2.0, (hall[2] + hall[3]) / 2.0, hall[1] - hall[0], hall[3] - hall[2]);
        for (size_t i = 0; i + 3 < boxes.size(); i += 4)
        {
            addRectangle(boxes[i], boxes[i + 1], boxes[i + 2], boxes[i + 3]);
        }

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        truth_sub_ = this->create_subscription<ferrari_vehicle::msg::OdomLite>("ground_truth", 10, [this](const ferrari_vehicle::msg::OdomLite::SharedPtr msg)
                                                                              {
            x_ = msg->pose.position.x;
            y_ = msg->pose.position.y;
            const auto &q = msg->pose.orientation;
            yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            has_pose_ = true; });
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), std::bind(&LidarSimulatorNode::publishScan, this));
    }

private:
    void addRectangle(double cx, double cy, double w, double h)
    {
        const double x0 = cx - w / 2.0, x1 = cx + w / 2.0, y0 = cy - h / 2.0, y1 = cy + h / 2.0;
        segments_.push_back({x0, y0, x1, y0});
        segments_.push_back({x1, y0, x1, y1});
        segments_.push_back({x1, y1, x0, y1});
        segments_.push_back({x0, y1, x0, y0});
    }

    double raycast(double ox, double oy, double angle) const
    {
        const double dx = std::cos(angle), dy = std::sin(angle);
        double best = std::numeric_limits<double>::infinity();
        for (const auto &s : segments_)
        {
            const double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
            const double denominator = dx * ey - dy * ex;
            if (std::abs(denominator) < 1e-12)
            {
                continue;
            }
            const double t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / denominator;
            const double u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / denominator;
            if (t > 0.0 && u >= 0.0 && u <= 1.0 && t < best)
            {
                best = t;
            }
        }
        return best;
    }

    void publishScan()
    {
        if (!has_pose_)
        {
            return;
        }

        const double cos_yaw = std::cos(yaw_), sin_yaw = std::sin(yaw_);
        const double ox = x_ + cos_yaw * lidar_x_ - sin_yaw * lidar_y_;
        const double oy = y_ + sin_yaw * lidar_x_ + cos_yaw * lidar_y_;

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->now();
        scan.header.frame_id = frame_id_;
        scan.angle_min = static_cast<float>(-M_PI);
        scan.angle_increment = static_cast<float>(2.0 * M_PI / bins_);
        scan.angle_max = static_cast<float>(-M_PI + scan.angle_increment * (bins_ - 1));
        scan.scan_time = 0.1f;
        scan.range_min = static_cast<float>(range_min_);
        scan.range_max = static_cast<float>(range_max_);
        scan.ranges.resize(bins_);
        scan.intensities.assign(bins_, 100.0f);

        std::normal_distribution<double> noise(0.0, noise_std_ > 0.0 ? noise_std_ : 1.0);
        for (int i = 0; i < bins_; ++i)
        {
            const double beam = -M_PI + (i + 0.5) * scan.angle_increment;
            double range = raycast(ox, oy, yaw_ + lidar_yaw_ + beam);
            if (std::isfinite(range) && noise_std_ > 0.0)
            {
                range += noise(rng_);
            }
            scan.ranges[i] = (range >= range_min_ && range <= range_max_) ? static_cast<float>(range) : std::numeric_limits<float>::infinity();
        }
        scan_pub_->publish(scan);
    }

    std::vector<Segment> segments_;
    double lidar_x_ = 0.0, lidar_y_ = 0.0, lidar_yaw_ = 0.0;
    int bins_ = 450;
    double range_min_ = 0.02, range_max_ = 12.0, noise_std_ = 0.01;
    std::string frame_id_;
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    bool has_pose_ = false;
    std::mt19937 rng_{7};

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Subscription<ferrari_vehicle::msg::OdomLite>::SharedPtr truth_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
