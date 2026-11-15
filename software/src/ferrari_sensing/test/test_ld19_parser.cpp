#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <gtest/gtest.h>

#include "ferrari_sensing/ld19_parser.hpp"

using namespace ferrari_sensing;

namespace
{
    uint8_t referenceCrc(const std::vector<uint8_t> &bytes)
    {
        uint8_t crc = 0;
        for (uint8_t byte : bytes)
        {
            crc ^= byte;
            for (int bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D) : static_cast<uint8_t>(crc << 1);
            }
        }
        return crc;
    }

    void putU16(std::vector<uint8_t> &out, uint16_t v)
    {
        out.push_back(static_cast<uint8_t>(v & 0xFF));
        out.push_back(static_cast<uint8_t>(v >> 8));
    }

    std::vector<uint8_t> encodePacket(double start_deg, double end_deg, const std::vector<std::pair<uint16_t, uint8_t>> &points,
                                      uint16_t speed = 3600, uint16_t timestamp = 1234)
    {
        std::vector<uint8_t> bytes = {kLd19Header, kLd19VerLen};
        putU16(bytes, speed);
        putU16(bytes, static_cast<uint16_t>(std::lround(start_deg * 100.0)));
        for (const auto &p : points)
        {
            putU16(bytes, p.first);
            bytes.push_back(p.second);
        }
        putU16(bytes, static_cast<uint16_t>(std::lround(end_deg * 100.0)));
        putU16(bytes, timestamp);
        bytes.push_back(referenceCrc(bytes));
        return bytes;
    }

    std::vector<std::pair<uint16_t, uint8_t>> rampPoints()
    {
        std::vector<std::pair<uint16_t, uint8_t>> points;
        for (int i = 0; i < 12; ++i)
        {
            points.emplace_back(static_cast<uint16_t>(1000 + 100 * i), static_cast<uint8_t>(10 * i));
        }
        return points;
    }

    std::vector<Ld19Packet> parseAll(Ld19Parser &parser, const std::vector<uint8_t> &bytes)
    {
        return parser.feed(bytes.data(), bytes.size());
    }
}

TEST(Ld19Crc, MatchesIndependentBitwiseImplementation)
{
    std::vector<uint8_t> data;
    for (int i = 0; i < 300; ++i)
    {
        data.push_back(static_cast<uint8_t>((i * 37 + 11) & 0xFF));
    }
    for (size_t length : {1u, 5u, 46u, 300u})
    {
        const std::vector<uint8_t> prefix(data.begin(), data.begin() + length);
        EXPECT_EQ(ld19Crc8(prefix.data(), prefix.size()), referenceCrc(prefix));
    }
}

TEST(Ld19Crc, AppendingCrcYieldsZero)
{
    std::vector<uint8_t> packet = encodePacket(10.0, 14.4, rampPoints());
    EXPECT_EQ(packet.size(), kLd19PacketSize);
    EXPECT_EQ(ld19Crc8(packet.data(), packet.size()), 0);
}

TEST(Ld19Parser, DecodesFieldsAndInterpolatesAngles)
{
    Ld19Parser parser;
    const auto packets = parseAll(parser, encodePacket(10.0, 14.4, rampPoints(), 3600, 777));

    ASSERT_EQ(packets.size(), 1u);
    const Ld19Packet &p = packets[0];
    EXPECT_DOUBLE_EQ(p.speed_deg_per_s, 3600.0);
    EXPECT_EQ(p.timestamp_ms, 777);
    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_NEAR(p.points[i].angle_deg, 10.0 + 0.4 * i, 1e-9);
        EXPECT_EQ(p.points[i].distance_mm, 1000 + 100 * i);
        EXPECT_EQ(p.points[i].intensity, 10 * i);
    }
}

TEST(Ld19Parser, HandlesAngleWrapAround)
{
    Ld19Parser parser;
    const auto packets = parseAll(parser, encodePacket(358.0, 2.2, rampPoints()));

    ASSERT_EQ(packets.size(), 1u);
    EXPECT_NEAR(packets[0].points.front().angle_deg, 358.0, 1e-9);
    EXPECT_NEAR(packets[0].points[5].angle_deg, std::fmod(358.0 + 5 * (4.2 / 11.0), 360.0), 1e-9);
    EXPECT_NEAR(packets[0].points.back().angle_deg, 2.2, 1e-9);
}

TEST(Ld19Parser, DiscardsCorruptedPacketsAndResynchronizes)
{
    std::vector<uint8_t> stream = {0x00, 0x54, 0x13, 0xFF, 0x54, 0x2C, 0x01};
    const auto good1 = encodePacket(10.0, 14.4, rampPoints());
    auto corrupted = encodePacket(20.0, 24.4, rampPoints());
    corrupted[20] ^= 0x40;
    const auto good2 = encodePacket(30.0, 34.4, rampPoints());

    stream.insert(stream.end(), good1.begin(), good1.end());
    stream.insert(stream.end(), corrupted.begin(), corrupted.end());
    stream.insert(stream.end(), good2.begin(), good2.end());

    Ld19Parser parser;
    const auto packets = parseAll(parser, stream);

    ASSERT_EQ(packets.size(), 2u);
    EXPECT_NEAR(packets[0].points[0].angle_deg, 10.0, 1e-9);
    EXPECT_NEAR(packets[1].points[0].angle_deg, 30.0, 1e-9);
    EXPECT_GE(parser.crcErrorCount(), 1u);
}

TEST(Ld19Parser, ReassemblesPacketsSplitAcrossReads)
{
    std::vector<uint8_t> stream;
    for (double start : {10.0, 20.0, 30.0})
    {
        const auto packet = encodePacket(start, start + 4.4, rampPoints());
        stream.insert(stream.end(), packet.begin(), packet.end());
    }

    Ld19Parser parser;
    size_t count = 0;
    for (uint8_t byte : stream)
    {
        count += parser.feed(&byte, 1).size();
    }
    EXPECT_EQ(count, 3u);
    EXPECT_EQ(parser.crcErrorCount(), 0u);
}

namespace
{
    std::vector<Ld19Packet> simulateRevolutions(int revolutions, const std::function<uint16_t(double)> &distance_mm)
    {
        Ld19Parser parser;
        std::vector<Ld19Packet> packets;
        constexpr int packets_per_revolution = 40;
        constexpr double step = 360.0 / (packets_per_revolution * 12);
        for (int r = 0; r < revolutions; ++r)
        {
            for (int k = 0; k < packets_per_revolution; ++k)
            {
                std::vector<std::pair<uint16_t, uint8_t>> points;
                const double start = (k * 12) * step;
                for (int i = 0; i < 12; ++i)
                {
                    points.emplace_back(distance_mm(start + i * step), 100);
                }
                const double end = start + 11 * step;
                const auto bytes = encodePacket(start, end, points, 3600);
                const auto parsed = parser.feed(bytes.data(), bytes.size());
                packets.insert(packets.end(), parsed.begin(), parsed.end());
            }
        }
        return packets;
    }

    std::vector<Scan> assemble(const std::vector<Ld19Packet> &packets, const ScanParams &params)
    {
        ScanAssembler assembler(params);
        std::vector<Scan> scans;
        for (const auto &packet : packets)
        {
            if (auto scan = assembler.addPacket(packet))
            {
                scans.push_back(*scan);
            }
        }
        return scans;
    }

    float rangeAt(const Scan &scan, double ros_angle)
    {
        const int bin = static_cast<int>(std::floor((ros_angle - scan.angle_min) / scan.angle_increment));
        return scan.ranges[std::clamp(bin, 0, static_cast<int>(scan.ranges.size()) - 1)];
    }
}

TEST(ScanAssembler, EmitsOneScanPerRevolutionAndDropsThePartialFirst)
{
    const auto packets = simulateRevolutions(4, [](double) -> uint16_t
                                             { return 2000; });
    const auto scans = assemble(packets, ScanParams());

    EXPECT_EQ(scans.size(), 2u);
    for (const auto &scan : scans)
    {
        EXPECT_NEAR(scan.scan_time_s, 0.1, 1e-9);
        EXPECT_EQ(scan.ranges.size(), 450u);
    }
}

TEST(ScanAssembler, ConvertsClockwiseLidarAnglesToCounterClockwiseRos)
{
    const auto packets = simulateRevolutions(3, [](double angle_deg) -> uint16_t
                                             { return (angle_deg > 85.0 && angle_deg < 95.0) ? 1000 : 2000; });
    const auto scans = assemble(packets, ScanParams());

    ASSERT_FALSE(scans.empty());
    const Scan &scan = scans.back();
    EXPECT_NEAR(rangeAt(scan, -M_PI / 2.0), 1.0, 1e-3);
    EXPECT_NEAR(rangeAt(scan, M_PI / 2.0), 2.0, 1e-3);
    EXPECT_NEAR(rangeAt(scan, 0.0), 2.0, 1e-3);
}

TEST(ScanAssembler, AppliesMountingAngleOffset)
{
    ScanParams params;
    params.angle_offset_rad = M_PI / 2.0;
    const auto packets = simulateRevolutions(3, [](double angle_deg) -> uint16_t
                                             { return (angle_deg > 85.0 && angle_deg < 95.0) ? 1000 : 2000; });
    const auto scans = assemble(packets, params);

    ASSERT_FALSE(scans.empty());
    EXPECT_NEAR(rangeAt(scans.back(), 0.0), 1.0, 1e-3);
}

TEST(ScanAssembler, MarksInvalidAndOutOfRangePointsAsInfinite)
{
    const auto packets = simulateRevolutions(3, [](double angle_deg) -> uint16_t
                                             {
        if (angle_deg < 90.0) return 0;
        if (angle_deg < 180.0) return 13000;
        if (angle_deg < 270.0) return 10;
        return 3000; });
    const auto scans = assemble(packets, ScanParams());

    ASSERT_FALSE(scans.empty());
    const Scan &scan = scans.back();
    EXPECT_TRUE(std::isinf(rangeAt(scan, -M_PI / 4.0)));
    EXPECT_TRUE(std::isinf(rangeAt(scan, -3.0 * M_PI / 4.0)));
    EXPECT_TRUE(std::isinf(rangeAt(scan, 3.0 * M_PI / 4.0)));
    EXPECT_NEAR(rangeAt(scan, M_PI / 4.0), 3.0, 1e-3);
}
