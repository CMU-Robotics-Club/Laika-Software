// Runs LaikaUsbHardwareInterface against a pseudo-terminal instead of the STM32:
// the plugin writes to the pty's slave side, the test reads the master side and
// decodes the frames the way the firmware does. No board and no ROS tools needed.
#include <gtest/gtest.h>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "laika_hardware_interface/laika_usb_hardware_interface.hpp"
#include "laika_hardware_interface/quads_usb_protocol.hpp"

using laika_hardware_interface::LaikaUsbHardwareInterface;
using namespace laika_hardware_interface::quads_usb;
using hardware_interface::CallbackReturn;

namespace
{

// Built by hand instead of parsing a URDF, so the test does not depend on the URDF
// parser. Same content as config/usb_leg_hardware_interface_config.xacro.
hardware_interface::HardwareComponentInterfaceParams make_params(
  const std::string & port, const std::string & hip_index = "1",
  const std::string & knee_index = "0")
{
  auto iface = [](const std::string & name, const std::string & min = "",
      const std::string & max = "") {
      hardware_interface::InterfaceInfo i;
      i.name = name;
      i.min = min;
      i.max = max;
      return i;
    };
  auto joint = [&](const std::string & name, const std::string & idx) {
      hardware_interface::ComponentInfo j;
      j.name = name;
      j.type = "joint";
      j.parameters["usb_index"] = idx;
      j.command_interfaces = {iface("position", "-0.5", "0.5"), iface("velocity"), iface("effort")};
      j.state_interfaces = {iface("position"), iface("velocity")};
      return j;
    };

  hardware_interface::HardwareComponentInterfaceParams p;
  auto & info = p.hardware_info;
  info.name = "usb";
  info.type = "system";
  info.hardware_plugin_name = "laika_hardware_interface/LaikaUsbHardwareInterface";
  info.hardware_parameters["serial_port"] = port;
  info.hardware_parameters["send_rate_hz"] = "100";
  info.joints = {joint("hip_joint", hip_index), joint("knee_joint", knee_index)};
  return p;
}

// Decoded CMD_JOINTS frame
struct Frame
{
  uint8_t type;
  uint16_t seq;
  JointsCommand joints;
};

class UsbHardwareTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char name[128];
    ASSERT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0);
    slave_name_ = name;
    fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK);
  }

  void TearDown() override
  {
    close(slave_);
    close(master_);
  }

  // Bring the plugin to "active" and grab its command handles
  void activate()
  {
    ASSERT_EQ(hw_.on_init(make_params(slave_name_)), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    commands_ = hw_.export_command_interfaces();
  }

  void set(const std::string & name, double v)
  {
    for (auto & c : commands_) {
      if (c.get_name() == name) {
        ASSERT_TRUE(c.set_value(v, true));
        return;
      }
    }
    FAIL() << "no command interface " << name;
  }

  void write_at(double t_s)
  {
    ASSERT_EQ(hw_.write(rclcpp::Time(static_cast<int64_t>(t_s * 1e9)), rclcpp::Duration(0, 0)),
      hardware_interface::return_type::OK);
  }

  // Everything the plugin has written so far, split at 0x00 and decoded
  std::vector<Frame> read_frames()
  {
    std::vector<uint8_t> bytes;
    uint8_t buf[512];
    long r;
    while ((r = ::read(master_, buf, sizeof(buf))) > 0) {
      bytes.insert(bytes.end(), buf, buf + r);
    }
    std::vector<Frame> frames;
    std::vector<uint8_t> chunk;
    for (uint8_t b : bytes) {
      if (b != 0) {chunk.push_back(b); continue;}
      uint8_t raw[kMaxFrameSize];
      const size_t m = cobs_decode(chunk.data(), chunk.size(), raw);
      chunk.clear();
      EXPECT_EQ(m, kHeaderSize + kJointsPayloadSize + kCrcSize);
      if (m != kHeaderSize + kJointsPayloadSize + kCrcSize) {continue;}
      EXPECT_EQ(raw[m - 2] | (raw[m - 1] << 8), crc16(raw, m - 2)) << "bad CRC";
      Frame f;
      f.type = raw[0];
      f.seq = static_cast<uint16_t>(raw[1] | (raw[2] << 8));
      std::memcpy(f.joints.data(), &raw[kHeaderSize], kJointsPayloadSize);  // x86 is little-endian
      frames.push_back(f);
    }
    EXPECT_TRUE(chunk.empty()) << "bytes left after the last 0x00";
    return frames;
  }

  int master_ = -1;
  int slave_ = -1;
  std::string slave_name_;
  LaikaUsbHardwareInterface hw_;
  std::vector<hardware_interface::CommandInterface> commands_;
};

}  // namespace

TEST_F(UsbHardwareTest, SendsNothingBeforeTargets)
{
  activate();
  write_at(1.0);
  write_at(1.1);
  EXPECT_TRUE(read_frames().empty());
}

TEST_F(UsbHardwareTest, PutsJointsAtTheirUsbIndex)
{
  activate();
  set("hip_joint/position", 0.3);
  set("knee_joint/position", 0.2);
  set("knee_joint/velocity", 1.0);
  set("knee_joint/effort", 0.25);
  write_at(1.0);

  const auto frames = read_frames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].type, kCmdJoints);
  const auto & j = frames[0].joints;
  // knee usb_index 0, hip usb_index 1 (firmware KNEE_INDEX / HIP_INDEX)
  EXPECT_FLOAT_EQ(j[0].q_des, 0.2f);
  EXPECT_FLOAT_EQ(j[0].qd_des, 1.0f);
  EXPECT_FLOAT_EQ(j[0].tau_ff, 0.25f);
  EXPECT_FLOAT_EQ(j[1].q_des, 0.3f);
  EXPECT_FLOAT_EQ(j[1].qd_des, 0.0f);
  for (size_t i = 2; i < kMaxJoints; ++i) {
    EXPECT_EQ(j[i].q_des, 0.0f) << "unused slot " << i;
    EXPECT_EQ(j[i].qd_des, 0.0f);
    EXPECT_EQ(j[i].tau_ff, 0.0f);
  }
}

TEST_F(UsbHardwareTest, ClampsPositionToLimits)
{
  activate();
  set("hip_joint/position", 2.0);
  set("knee_joint/position", -3.0);
  write_at(1.0);
  const auto frames = read_frames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_FLOAT_EQ(frames[0].joints[1].q_des, 0.5f);
  EXPECT_FLOAT_EQ(frames[0].joints[0].q_des, -0.5f);
}

TEST_F(UsbHardwareTest, SendsAtSendRateWithIncreasingSeq)
{
  activate();
  set("hip_joint/position", 0.1);
  set("knee_joint/position", 0.1);
  // 100 Hz send rate: of these writes only t = 1.000, 1.010 and 1.020 send
  for (double t : {1.000, 1.001, 1.005, 1.010, 1.015, 1.020}) {
    write_at(t);
  }
  const auto frames = read_frames();
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(static_cast<uint16_t>(frames[1].seq - frames[0].seq), 1);
  EXPECT_EQ(static_cast<uint16_t>(frames[2].seq - frames[1].seq), 1);
}

TEST_F(UsbHardwareTest, StopsSendingWhenDeactivated)
{
  activate();
  set("hip_joint/position", 0.1);
  set("knee_joint/position", 0.1);
  ASSERT_EQ(hw_.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  write_at(1.0);
  EXPECT_TRUE(read_frames().empty());
}

TEST_F(UsbHardwareTest, RejectsBadUsbIndex)
{
  EXPECT_EQ(hw_.on_init(make_params(slave_name_, "1", "1")), CallbackReturn::ERROR);
}

TEST_F(UsbHardwareTest, RejectsOutOfRangeUsbIndex)
{
  EXPECT_EQ(hw_.on_init(make_params(slave_name_, "12", "0")), CallbackReturn::ERROR);
}

TEST_F(UsbHardwareTest, ConfigureFailsWithoutPort)
{
  ASSERT_EQ(hw_.on_init(make_params("/dev/does_not_exist")), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}
