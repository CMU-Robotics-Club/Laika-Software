// Framing for the Quads USB protocol (Jetson -> STM32 over USB CDC).
//
// Wire format (see quads_firmware doc/usb_protocol.md, branch feat/usb):
//   COBS( type u8 | seq u16 | len u8 | payload | crc16 u16 ) 0x00
// Little-endian. CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, no xorout.
//
// Must stay in sync with quads_firmware Core/Inc/usb/usb_msgs.h and
// scripts/quads_host.py. Pure functions, no ROS and no heap use, so they are
// safe to call from the ros2_control write() loop.
#ifndef LAIKA_HARDWARE_INTERFACE__QUADS_USB_PROTOCOL_HPP_
#define LAIKA_HARDWARE_INTERFACE__QUADS_USB_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace laika_hardware_interface::quads_usb
{

// Message types (type byte on the wire)
constexpr uint8_t kCmdJoints = 0x01;

// The protocol always carries 12 joints; unused ones are sent as 0
constexpr size_t kMaxJoints = 12;

constexpr size_t kHeaderSize = 4;    // type(1) + seq(2) + len(1)
constexpr size_t kCrcSize = 2;
constexpr size_t kMaxPayload = 255;  // len is one byte
constexpr size_t kJointsPayloadSize = kMaxJoints * 3 * sizeof(float);  // 144

// COBS adds at most one byte per 254 input bytes (plus one), then the 0x00 delimiter
constexpr size_t cobs_max_encoded_size(size_t n) {return n + n / 254 + 1;}
constexpr size_t kMaxFrameSize =
  cobs_max_encoded_size(kHeaderSize + kMaxPayload + kCrcSize) + 1;  // 264

// One joint's setpoint, joint side: rad, rad/s, N*m
struct JointSetpoint
{
  float q_des = 0.0f;
  float qd_des = 0.0f;
  float tau_ff = 0.0f;
};

using JointsCommand = std::array<JointSetpoint, kMaxJoints>;

// CRC16-CCITT-FALSE over `n` bytes
uint16_t crc16(const uint8_t * data, size_t n);

// COBS-encode `n` bytes into `out` (no trailing 0x00).
// `out` must hold cobs_max_encoded_size(n) bytes. Returns the encoded length.
size_t cobs_encode(const uint8_t * in, size_t n, uint8_t * out);

// COBS-decode `n` bytes (without the trailing 0x00) into `out` (at least n bytes).
// Returns the decoded length, or 0 if the input is not valid COBS.
size_t cobs_decode(const uint8_t * in, size_t n, uint8_t * out);

// Build a complete frame, including the trailing 0x00, into `out`.
// Returns the frame length, or 0 if `len` > kMaxPayload or `out_cap` is too small.
size_t build_frame(
  uint8_t type, uint16_t seq, const uint8_t * payload, size_t len,
  uint8_t * out, size_t out_cap);

// Pack 12 x (q_des, qd_des, tau_ff) as little-endian f32 into a 144-byte payload
void pack_joints(const JointsCommand & cmd, uint8_t * out);

}  // namespace laika_hardware_interface::quads_usb

#endif  // LAIKA_HARDWARE_INTERFACE__QUADS_USB_PROTOCOL_HPP_
