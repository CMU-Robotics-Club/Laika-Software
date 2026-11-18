#include "laika_hardware_interface/usb_serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace laika_hardware_interface
{

UsbSerial::~UsbSerial()
{
  close();
}

bool UsbSerial::open(const std::string & path, std::string & error)
{
  close();

  // O_NOCTTY: the port must not become our controlling terminal (Ctrl-C etc.)
  // O_NONBLOCK: write() returns at once instead of stalling the control loop
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = "open(" + path + "): " + std::strerror(errno);
    return false;
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  // Raw mode: no echo, no line buffering, no byte translation (0x0D, 0x03, 0x11 ...).
  // Without it the tty layer rewrites bytes and frames fail their CRC.
  cfmakeraw(&tio);
  // USB CDC ignores the baud rate, but the field must hold a valid value
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= CLOCAL | CREAD;  // ignore modem lines, enable the receiver
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);  // drop anything left from before we opened
  return true;
}

void UsbSerial::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

long UsbSerial::write_some(const uint8_t * data, size_t n)
{
  if (fd_ < 0) {
    errno = EBADF;
    return -1;
  }
  const ssize_t w = ::write(fd_, data, n);
  if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return 0;  // OS buffer full: nothing written this time
  }
  return static_cast<long>(w);
}

}  // namespace laika_hardware_interface
