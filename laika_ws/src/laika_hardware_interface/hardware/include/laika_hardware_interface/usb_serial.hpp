// Minimal raw, non-blocking serial port for the STM32's USB CDC device (Linux).
#ifndef LAIKA_HARDWARE_INTERFACE__USB_SERIAL_HPP_
#define LAIKA_HARDWARE_INTERFACE__USB_SERIAL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace laika_hardware_interface
{

class UsbSerial
{
public:
  UsbSerial() = default;
  ~UsbSerial();
  UsbSerial(const UsbSerial &) = delete;
  UsbSerial & operator=(const UsbSerial &) = delete;

  // Open `path` in raw mode (cfmakeraw), non-blocking. On failure returns false
  // and puts the reason in `error`.
  bool open(const std::string & path, std::string & error);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Write without blocking. Returns the number of bytes written (may be less than
  // `n` if the OS buffer is full), or -1 on error (errno is set).
  long write_some(const uint8_t * data, size_t n);

private:
  int fd_ = -1;
};

}  // namespace laika_hardware_interface

#endif  // LAIKA_HARDWARE_INTERFACE__USB_SERIAL_HPP_
