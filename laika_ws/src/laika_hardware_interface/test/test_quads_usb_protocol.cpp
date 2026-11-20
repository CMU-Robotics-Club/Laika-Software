// Expected values come from scripts/quads_host.py (quads_firmware, branch feat/usb),
// so these tests check that the C++ framing matches the reference implementation.
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "laika_hardware_interface/quads_usb_protocol.hpp"

using namespace laika_hardware_interface::quads_usb;

namespace
{
std::vector<uint8_t> bytes(std::initializer_list<int> v)
{
  return std::vector<uint8_t>(v.begin(), v.end());
}
}  // namespace

TEST(Crc16, StandardCheckValue)
{
  // Published check value for CRC-16/CCITT-FALSE
  const char * s = "123456789";
  EXPECT_EQ(crc16(reinterpret_cast<const uint8_t *>(s), 9), 0x29B1);
}

TEST(Crc16, MatchesPythonExample)
{
  const auto body = bytes({0x7e, 0x05, 0x00, 0x02, 0x01, 0x02});
  EXPECT_EQ(crc16(body.data(), body.size()), 0x956B);
}

TEST(Cobs, EncodesExample)
{
  const auto raw = bytes({0x7e, 0x05, 0x00, 0x02, 0x01, 0x02, 0x6b, 0x95});
  uint8_t out[16];
  const size_t n = cobs_encode(raw.data(), raw.size(), out);
  EXPECT_EQ(std::vector<uint8_t>(out, out + n),
    bytes({0x03, 0x7e, 0x05, 0x06, 0x02, 0x01, 0x02, 0x6b, 0x95}));
}

TEST(Cobs, EncodesConsecutiveZeros)
{
  const auto raw = bytes({0x11, 0x00, 0x00, 0x22});
  uint8_t out[16];
  const size_t n = cobs_encode(raw.data(), raw.size(), out);
  EXPECT_EQ(std::vector<uint8_t>(out, out + n), bytes({0x02, 0x11, 0x01, 0x02, 0x22}));
}

TEST(Cobs, RoundTripsLongBlockWithoutZeros)
{
  // 300 non-zero bytes forces the 0xFF (254-byte block) path
  std::vector<uint8_t> raw(300);
  for (size_t i = 0; i < raw.size(); ++i) {
    raw[i] = static_cast<uint8_t>(i % 255 + 1);
  }
  std::vector<uint8_t> enc(cobs_max_encoded_size(raw.size()));
  const size_t n = cobs_encode(raw.data(), raw.size(), enc.data());
  for (size_t i = 0; i < n; ++i) {
    ASSERT_NE(enc[i], 0) << "encoded data must not contain 0x00 (index " << i << ")";
  }
  std::vector<uint8_t> dec(n);
  const size_t m = cobs_decode(enc.data(), n, dec.data());
  EXPECT_EQ(std::vector<uint8_t>(dec.begin(), dec.begin() + m), raw);
}

TEST(Cobs, RejectsZeroCodeByte)
{
  // Second code byte is 0x00: like quads_host.py, decode must fail
  const auto bad = bytes({0x02, 0x7e, 0x00, 0x01});
  uint8_t out[8];
  EXPECT_EQ(cobs_decode(bad.data(), bad.size(), out), 0u);
}

TEST(Cobs, RejectsBlockPastEnd)
{
  // Code 0x05 promises 4 data bytes but only 2 follow (truncated frame)
  const auto bad = bytes({0x05, 0x7e, 0x05});
  uint8_t out[8];
  EXPECT_EQ(cobs_decode(bad.data(), bad.size(), out), 0u);
}

TEST(Frame, MatchesPythonExample)
{
  // type 0x7E, seq 5, payload 01 02 -> 03 7e 05 06 02 01 02 6b 95 00
  const auto payload = bytes({0x01, 0x02});
  uint8_t out[kMaxFrameSize];
  const size_t n = build_frame(0x7E, 5, payload.data(), payload.size(), out, sizeof(out));
  EXPECT_EQ(std::vector<uint8_t>(out, out + n),
    bytes({0x03, 0x7e, 0x05, 0x06, 0x02, 0x01, 0x02, 0x6b, 0x95, 0x00}));
}

TEST(Frame, JointsFrameIs152BytesAndDecodes)
{
  JointsCommand cmd{};
  cmd[0] = {0.3f, -1.5f, 0.25f};
  cmd[1] = {0.5f, 0.0f, 0.0f};

  uint8_t payload[kJointsPayloadSize];
  pack_joints(cmd, payload);

  uint8_t frame[kMaxFrameSize];
  const size_t n = build_frame(kCmdJoints, 0xBEEF, payload, sizeof(payload), frame, sizeof(frame));
  ASSERT_EQ(n, 152u);
  EXPECT_EQ(frame[n - 1], 0x00);  // delimiter
  for (size_t i = 0; i + 1 < n; ++i) {
    ASSERT_NE(frame[i], 0x00) << "only the last byte may be 0x00 (index " << i << ")";
  }

  // Undo the framing like the firmware does and check every field
  uint8_t raw[kMaxFrameSize];
  const size_t m = cobs_decode(frame, n - 1, raw);
  ASSERT_EQ(m, kHeaderSize + kJointsPayloadSize + kCrcSize);
  EXPECT_EQ(raw[0], kCmdJoints);
  EXPECT_EQ(raw[1] | (raw[2] << 8), 0xBEEF);  // seq, little-endian
  EXPECT_EQ(raw[3], kJointsPayloadSize);
  EXPECT_EQ(raw[m - 2] | (raw[m - 1] << 8), crc16(raw, m - 2));

  // Joint 0 q_des sits at payload offset 0, joint 1 q_des at offset 12
  float q0, qd0, q1;
  std::memcpy(&q0, &raw[kHeaderSize + 0], 4);
  std::memcpy(&qd0, &raw[kHeaderSize + 4], 4);
  std::memcpy(&q1, &raw[kHeaderSize + 12], 4);
  EXPECT_FLOAT_EQ(q0, 0.3f);
  EXPECT_FLOAT_EQ(qd0, -1.5f);
  EXPECT_FLOAT_EQ(q1, 0.5f);
}

TEST(Frame, RejectsOversizedPayload)
{
  uint8_t payload[kMaxPayload + 1] = {};
  uint8_t out[kMaxFrameSize + 8];
  EXPECT_EQ(build_frame(kCmdJoints, 0, payload, sizeof(payload), out, sizeof(out)), 0u);
}
