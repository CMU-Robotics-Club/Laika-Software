#include "laika_hardware_interface/quads_usb_protocol.hpp"

#include <cstring>

namespace laika_hardware_interface::quads_usb
{

namespace
{

// Write a value byte by byte, low byte first, so the result is little-endian
// whatever the CPU's byte order is
void put_u16_le(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}

void put_f32_le(uint8_t * p, float f)
{
  uint32_t v;
  std::memcpy(&v, &f, sizeof(v));  // reinterpret the float's bits without UB
  for (int i = 0; i < 4; ++i) {
    p[i] = static_cast<uint8_t>(v >> (8 * i));
  }
}

}  // namespace

uint16_t crc16(const uint8_t * data, size_t n)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;  // next byte into the high 8 bits
    for (int bit = 0; bit < 8; ++bit) {
      // Top bit set: shift and subtract (XOR) the polynomial. uint16_t drops the overflow.
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t cobs_encode(const uint8_t * in, size_t n, uint8_t * out)
{
  size_t out_len = 1;     // out[0] is reserved for the first code byte
  size_t code_idx = 0;    // where the current block's code byte goes
  uint8_t code = 1;       // block length so far, counting the code byte itself

  for (size_t i = 0; i < n; ++i) {
    if (in[i] == 0) {
      // A zero ends the block: write its code, start a new block
      out[code_idx] = code;
      code_idx = out_len++;
      code = 1;
    } else {
      out[out_len++] = in[i];
      if (++code == 0xFF) {
        // 254 non-zero bytes: close the block without an implied zero
        out[code_idx] = code;
        code_idx = out_len++;
        code = 1;
      }
    }
  }
  out[code_idx] = code;
  return out_len;
}

size_t cobs_decode(const uint8_t * in, size_t n, uint8_t * out)
{
  size_t i = 0;
  size_t out_len = 0;
  while (i < n) {
    const uint8_t code = in[i++];
    if (code == 0 || i + code - 1 > n) {
      return 0;  // a 0x00 inside a frame, or a block running past the end
    }
    for (uint8_t k = 1; k < code; ++k) {
      out[out_len++] = in[i++];
    }
    // The block stood for "data + 0x00", except a full block or the last one
    if (code != 0xFF && i < n) {
      out[out_len++] = 0;
    }
  }
  return out_len;
}

size_t build_frame(
  uint8_t type, uint16_t seq, const uint8_t * payload, size_t len,
  uint8_t * out, size_t out_cap)
{
  if (len > kMaxPayload) {
    return 0;
  }
  const size_t raw_len = kHeaderSize + len + kCrcSize;
  if (out_cap < cobs_max_encoded_size(raw_len) + 1) {
    return 0;
  }

  // 1. header + payload
  uint8_t raw[kHeaderSize + kMaxPayload + kCrcSize];
  raw[0] = type;
  put_u16_le(&raw[1], seq);
  raw[3] = static_cast<uint8_t>(len);
  if (len > 0) {
    std::memcpy(&raw[kHeaderSize], payload, len);
  }

  // 2. CRC over header + payload, appended little-endian
  put_u16_le(&raw[kHeaderSize + len], crc16(raw, kHeaderSize + len));

  // 3. COBS, 4. delimiter
  const size_t enc_len = cobs_encode(raw, raw_len, out);
  out[enc_len] = 0x00;
  return enc_len + 1;
}

void pack_joints(const JointsCommand & cmd, uint8_t * out)
{
  // Joint i occupies bytes [12*i, 12*i + 12): q_des, qd_des, tau_ff
  for (size_t i = 0; i < kMaxJoints; ++i) {
    uint8_t * p = out + i * 3 * sizeof(float);
    put_f32_le(p + 0, cmd[i].q_des);
    put_f32_le(p + 4, cmd[i].qd_des);
    put_f32_le(p + 8, cmd[i].tau_ff);
  }
}

}  // namespace laika_hardware_interface::quads_usb
