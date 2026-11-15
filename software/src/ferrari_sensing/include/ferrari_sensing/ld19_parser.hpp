#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace ferrari_sensing
{
    constexpr size_t kLd19PointsPerPacket = 12;
    constexpr size_t kLd19PacketSize = 47;
    constexpr uint8_t kLd19Header = 0x54;
    constexpr uint8_t kLd19VerLen = 0x2C;

    struct Ld19Point
    {
        double angle_deg;
        uint16_t distance_mm;
        uint8_t intensity;
    };

    struct Ld19Packet
    {
        double speed_deg_per_s;
        uint16_t timestamp_ms;
        std::array<Ld19Point, kLd19PointsPerPacket> points;
    };

    uint8_t ld19Crc8(const uint8_t *data, size_t length);

    class Ld19Parser
    {
    public:
        std::vector<Ld19Packet> feed(const uint8_t *data, size_t length);

        uint64_t packetCount() const { return packet_count_; }
        uint64_t crcErrorCount() const { return crc_error_count_; }
        uint64_t discardedBytes() const { return discarded_bytes_; }

    private:
        std::vector<uint8_t> buffer_;
        uint64_t packet_count_ = 0;
        uint64_t crc_error_count_ = 0;
        uint64_t discarded_bytes_ = 0;
    };

    struct ScanParams
    {
        int bins = 450;
        double angle_offset_rad = 0.0;
        double range_min_m = 0.02;
        double range_max_m = 12.0;
        size_t min_points_per_scan = 100;
    };

    struct Scan
    {
        double angle_min = 0.0;
        double angle_increment = 0.0;
        double scan_time_s = 0.0;
        std::vector<float> ranges;
        std::vector<float> intensities;
        size_t valid_points = 0;
    };

    class ScanAssembler
    {
    public:
        explicit ScanAssembler(const ScanParams &params = ScanParams());

        std::optional<Scan> addPacket(const Ld19Packet &packet);

    private:
        void resetAccumulation();
        void addPoint(const Ld19Point &point);

        ScanParams params_;
        Scan current_;
        double last_angle_deg_ = -1.0;
        double speed_deg_per_s_ = 0.0;
        size_t point_count_ = 0;
        bool synchronized_ = false;
    };
}
