#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ferrari_sensing/ld19_parser.hpp"

using namespace ferrari_sensing;

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("lidar_node")
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("baudrate", 230400);
        frame_id_ = this->declare_parameter<std::string>("frame_id", "lidar_link");
        timeout_s_ = this->declare_parameter<double>("data_timeout_s", 1.0);

        ScanParams params;
        params.bins = this->declare_parameter<int>("scan_bins", params.bins);
        params.angle_offset_rad = this->declare_parameter<double>("angle_offset_rad", params.angle_offset_rad);
        params.range_min_m = this->declare_parameter<double>("range_min_m", params.range_min_m);
        params.range_max_m = this->declare_parameter<double>("range_max_m", params.range_max_m);
        params.min_points_per_scan = static_cast<size_t>(this->declare_parameter<int>("min_points_per_scan", static_cast<int>(params.min_points_per_scan)));
        range_min_ = params.range_min_m;
        range_max_ = params.range_max_m;
        assembler_ = std::make_unique<ScanAssembler>(params);

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        stats_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&LidarNode::reportStatistics, this));

        running_ = true;
        reader_ = std::thread(&LidarNode::readLoop, this);
    }

    ~LidarNode()
    {
        running_ = false;
        if (reader_.joinable())
        {
            reader_.join();
        }
        closePort();
    }

private:
    static speed_t baudConstant(int baudrate)
    {
        switch (baudrate)
        {
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        default:
            return B230400;
        }
    }

    bool openPort()
    {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            return false;
        }

        termios tty;
        if (tcgetattr(fd_, &tty) != 0)
        {
            closePort();
            return false;
        }
        cfmakeraw(&tty);
        cfsetispeed(&tty, baudConstant(baudrate_));
        cfsetospeed(&tty, baudConstant(baudrate_));
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            closePort();
            return false;
        }
        tcflush(fd_, TCIFLUSH);
        return true;
    }

    void closePort()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void readLoop()
    {
        uint8_t buffer[1024];
        auto last_data = std::chrono::steady_clock::now();

        while (running_)
        {
            if (fd_ < 0)
            {
                if (!openPort())
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot open %s: %s", port_.c_str(), std::strerror(errno));
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                RCLCPP_INFO(get_logger(), "Opened %s at %d baud", port_.c_str(), baudrate_);
                last_data = std::chrono::steady_clock::now();
            }

            pollfd descriptor{fd_, POLLIN, 0};
            const int ready = ::poll(&descriptor, 1, 100);
            if (ready > 0)
            {
                const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
                if (count > 0)
                {
                    last_data = std::chrono::steady_clock::now();
                    handleBytes(buffer, static_cast<size_t>(count));
                }
                else if (count < 0 && errno != EAGAIN && errno != EINTR)
                {
                    RCLCPP_WARN(get_logger(), "Read error: %s", std::strerror(errno));
                    closePort();
                }
            }

            const double silence = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_data).count();
            if (fd_ >= 0 && silence > timeout_s_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No data from lidar for %.1f s", silence);
            }
        }
    }

    void handleBytes(const uint8_t *data, size_t count)
    {
        for (const auto &packet : parser_.feed(data, count))
        {
            const auto scan = assembler_->addPacket(packet);
            if (scan)
            {
                publishScan(*scan);
            }
        }
    }

    void publishScan(const Scan &scan)
    {
        sensor_msgs::msg::LaserScan msg;
        const rclcpp::Time now = this->now();
        msg.header.stamp = now - rclcpp::Duration::from_seconds(scan.scan_time_s);
        msg.header.frame_id = frame_id_;
        msg.angle_min = static_cast<float>(scan.angle_min);
        msg.angle_increment = static_cast<float>(scan.angle_increment);
        msg.angle_max = static_cast<float>(scan.angle_min + scan.angle_increment * (scan.ranges.size() - 1));
        msg.scan_time = static_cast<float>(scan.scan_time_s);
        msg.time_increment = static_cast<float>(scan.scan_time_s / scan.ranges.size());
        msg.range_min = static_cast<float>(range_min_);
        msg.range_max = static_cast<float>(range_max_);
        msg.ranges = scan.ranges;
        msg.intensities = scan.intensities;
        scan_pub_->publish(msg);
        ++published_scans_;
    }

    void reportStatistics()
    {
        RCLCPP_INFO(get_logger(), "lidar: %lu scans, %lu packets, %lu crc errors, %lu bytes discarded",
                    static_cast<unsigned long>(published_scans_.load()), static_cast<unsigned long>(parser_.packetCount()),
                    static_cast<unsigned long>(parser_.crcErrorCount()), static_cast<unsigned long>(parser_.discardedBytes()));
    }

    std::string port_;
    int baudrate_ = 230400;
    std::string frame_id_;
    double timeout_s_ = 1.0;
    double range_min_ = 0.0;
    double range_max_ = 0.0;

    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread reader_;
    std::atomic<uint64_t> published_scans_{0};

    Ld19Parser parser_;
    std::unique_ptr<ScanAssembler> assembler_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}
