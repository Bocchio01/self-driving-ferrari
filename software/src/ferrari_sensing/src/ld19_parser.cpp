#include "ferrari_sensing/ld19_parser.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ferrari_sensing
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        const std::array<uint8_t, 256> &crcTable()
        {
            static const std::array<uint8_t, 256> table = []
            {
                std::array<uint8_t, 256> t{};
                for (int i = 0; i < 256; ++i)
                {
                    uint8_t crc = static_cast<uint8_t>(i);
                    for (int bit = 0; bit < 8; ++bit)
                    {
                        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D) : static_cast<uint8_t>(crc << 1);
                    }
                    t[i] = crc;
                }
                return t;
            }();
            return table;
        }

        uint16_t readU16(const uint8_t *p)
        {
            return static_cast<uint16_t>(p[0] | (p[1] << 8));
        }
    }

    uint8_t ld19Crc8(const uint8_t *data, size_t length)
    {
        uint8_t crc = 0;
        for (size_t i = 0; i < length; ++i)
        {
            crc = crcTable()[static_cast<uint8_t>(crc ^ data[i])];
        }
        return crc;
    }

    std::vector<Ld19Packet> Ld19Parser::feed(const uint8_t *data, size_t length)
    {
        buffer_.insert(buffer_.end(), data, data + length);

        std::vector<Ld19Packet> packets;
        size_t position = 0;
        while (buffer_.size() - position >= kLd19PacketSize)
        {
            const uint8_t *p = buffer_.data() + position;
            if (p[0] != kLd19Header || p[1] != kLd19VerLen)
            {
                ++position;
                ++discarded_bytes_;
                continue;
            }
            if (ld19Crc8(p, kLd19PacketSize - 1) != p[kLd19PacketSize - 1])
            {
                ++position;
                ++discarded_bytes_;
                ++crc_error_count_;
                continue;
            }

            Ld19Packet packet;
            packet.speed_deg_per_s = readU16(p + 2);
            const double start = readU16(p + 4) / 100.0;
            double end = readU16(p + 4 + 2 + 3 * kLd19PointsPerPacket) / 100.0;
            if (end < start)
            {
                end += 360.0;
            }
            packet.timestamp_ms = readU16(p + 4 + 2 + 3 * kLd19PointsPerPacket + 2);

            const double step = (end - start) / (kLd19PointsPerPacket - 1);
            for (size_t i = 0; i < kLd19PointsPerPacket; ++i)
            {
                const uint8_t *q = p + 6 + 3 * i;
                packet.points[i] = {std::fmod(start + step * i, 360.0), readU16(q), q[2]};
            }
            packets.push_back(packet);
            ++packet_count_;
            position += kLd19PacketSize;
        }

        buffer_.erase(buffer_.begin(), buffer_.begin() + position);
        return packets;
    }

    ScanAssembler::ScanAssembler(const ScanParams &params) : params_(params)
    {
        resetAccumulation();
    }

    void ScanAssembler::resetAccumulation()
    {
        current_ = Scan();
        current_.angle_min = -kPi;
        current_.angle_increment = 2.0 * kPi / params_.bins;
        current_.ranges.assign(params_.bins, std::numeric_limits<float>::infinity());
        current_.intensities.assign(params_.bins, 0.0f);
        point_count_ = 0;
    }

    void ScanAssembler::addPoint(const Ld19Point &point)
    {
        const double range = point.distance_mm / 1000.0;
        if (point.distance_mm == 0 || range < params_.range_min_m || range > params_.range_max_m)
        {
            return;
        }

        double angle = -point.angle_deg * kPi / 180.0 + params_.angle_offset_rad;
        angle = std::remainder(angle, 2.0 * kPi);
        int bin = static_cast<int>(std::floor((angle - current_.angle_min) / current_.angle_increment));
        bin = std::clamp(bin, 0, params_.bins - 1);

        if (range < current_.ranges[bin])
        {
            current_.ranges[bin] = static_cast<float>(range);
            current_.intensities[bin] = point.intensity;
        }
        ++current_.valid_points;
    }

    std::optional<Scan> ScanAssembler::addPacket(const Ld19Packet &packet)
    {
        std::optional<Scan> completed;
        speed_deg_per_s_ = packet.speed_deg_per_s;

        for (const auto &point : packet.points)
        {
            if (last_angle_deg_ >= 0.0 && point.angle_deg < last_angle_deg_ - 180.0)
            {
                if (synchronized_ && point_count_ >= params_.min_points_per_scan)
                {
                    current_.scan_time_s = speed_deg_per_s_ > 0.0 ? 360.0 / speed_deg_per_s_ : 0.0;
                    completed = std::move(current_);
                }
                resetAccumulation();
                synchronized_ = true;
            }
            last_angle_deg_ = point.angle_deg;
            ++point_count_;
            addPoint(point);
        }
        return completed;
    }
}
