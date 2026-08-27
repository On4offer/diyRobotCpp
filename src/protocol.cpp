#include "diyrobot/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#endif

namespace diyrobot {

#ifndef _WIN32
class PosixSerialStream final : public ByteStream {
 public:
  PosixSerialStream(const std::string& port, unsigned baudrate, unsigned timeout_ms)
      : timeout_ms_(timeout_ms) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw ProtocolError("cannot open serial port " + port + ": " + std::strerror(errno));
    }
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      fail("tcgetattr failed");
    }
    speed_t speed{};
    switch (baudrate) {
      case 115200:
        speed = B115200;
        break;
#ifdef B1000000
      case 1000000:
        speed = B1000000;
        break;
#endif
      default:
        fail("unsupported POSIX baud rate");
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag = static_cast<tcflag_t>((tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS));
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      fail("tcsetattr failed");
    }
  }
  ~PosixSerialStream() override {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  void clear_input() override {
    tcflush(fd_, TCIFLUSH);
  }
  void write(const std::vector<std::uint8_t>& bytes) override {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
      if (count <= 0) {
        throw ProtocolError("serial write failed");
      }
      offset += static_cast<std::size_t>(count);
    }
    tcdrain(fd_);
  }
  std::vector<std::uint8_t> read_exact(std::size_t size) override {
    std::vector<std::uint8_t> out(size);
    std::size_t offset = 0;
    while (offset < size) {
      pollfd descriptor{fd_, POLLIN, 0};
      if (::poll(&descriptor, 1, static_cast<int>(timeout_ms_)) <= 0) {
        throw ProtocolError("serial read timeout");
      }
      const auto count = ::read(fd_, out.data() + offset, size - offset);
      if (count <= 0) {
        throw ProtocolError("serial read failed");
      }
      offset += static_cast<std::size_t>(count);
    }
    return out;
  }

 private:
  [[noreturn]] void fail(const char* message) {
    ::close(fd_);
    fd_ = -1;
    throw ProtocolError(message);
  }
  int fd_{-1};
  unsigned timeout_ms_{};
};
#endif

std::uint8_t checksum(const std::vector<std::uint8_t>& body) {
  unsigned sum = 0;
  for (const auto value : body) {
    sum += value;
  }
  return static_cast<std::uint8_t>(~sum & 0xFFU);
}

std::vector<std::uint8_t> build_instruction_packet(int servo_id, int instruction,
                                                   const std::vector<std::uint8_t>& params) {
  if (servo_id < 0 || servo_id > kBroadcastId) {
    throw ProtocolError("invalid servo id");
  }
  if (instruction < 0 || instruction > 255 || params.size() > 251) {
    throw ProtocolError("invalid instruction packet");
  }
  std::vector<std::uint8_t> body{static_cast<std::uint8_t>(servo_id),
                                 static_cast<std::uint8_t>(params.size() + 2),
                                 static_cast<std::uint8_t>(instruction)};
  body.insert(body.end(), params.begin(), params.end());
  std::vector<std::uint8_t> packet{0xFF, 0xFF};
  packet.insert(packet.end(), body.begin(), body.end());
  packet.push_back(checksum(body));
  return packet;
}

StatusPacket parse_status_packet(const std::vector<std::uint8_t>& packet) {
  if (packet.size() < 6) {
    throw ProtocolError("status packet too short");
  }
  if (packet[0] != 0xFF || packet[1] != 0xFF) {
    throw ProtocolError("bad status header");
  }
  const auto length = packet[3];
  if (packet.size() != static_cast<std::size_t>(length) + 4U) {
    throw ProtocolError("bad status length");
  }
  std::vector<std::uint8_t> body(packet.begin() + 2, packet.end() - 1);
  if (packet.back() != checksum(body)) {
    throw ProtocolError("bad status checksum");
  }
  return {packet[2], packet[4], {packet.begin() + 5, packet.end() - 1}, length};
}

std::vector<StatusPacket> parse_status_stream(const std::vector<std::uint8_t>& raw) {
  std::vector<StatusPacket> result;
  std::size_t i = 0;
  while (i + 3 < raw.size()) {
    if (raw[i] != 0xFF || raw[i + 1] != 0xFF) {
      ++i;
      continue;
    }
    const std::size_t end = i + 4U + raw[i + 3];
    if (end > raw.size()) {
      break;
    }
    try {
      result.push_back(parse_status_packet({raw.begin() + static_cast<std::ptrdiff_t>(i),
                                            raw.begin() + static_cast<std::ptrdiff_t>(end)}));
    } catch (const ProtocolError&) {
      ++i;
      continue;
    }
    i = end;
  }
  return result;
}

std::uint16_t encode_sign_magnitude(int value, unsigned bit) {
  if (bit >= 16) {
    throw ProtocolError("sign bit out of range");
  }
  const unsigned magnitude = static_cast<unsigned>(std::abs(value));
  if (magnitude >= (1U << bit)) {
    throw ProtocolError("sign-magnitude overflow");
  }
  return static_cast<std::uint16_t>(value < 0 ? magnitude | (1U << bit) : magnitude);
}

int decode_sign_magnitude(std::uint16_t value, unsigned bit) {
  const auto mask = static_cast<std::uint16_t>(1U << bit);
  return (value & mask) ? -static_cast<int>(value & ~mask) : static_cast<int>(value);
}

std::pair<std::uint8_t, std::uint8_t> split_u16(std::uint16_t value) {
  return {static_cast<std::uint8_t>(value & 0xFFU),
          static_cast<std::uint8_t>((value >> 8U) & 0xFFU)};
}
std::uint16_t join_u16(std::uint8_t lo, std::uint8_t hi) {
  return static_cast<std::uint16_t>(lo | (static_cast<unsigned>(hi) << 8U));
}
int angle_to_counts(double degrees, int resolution) {
  return static_cast<int>(std::lround(degrees * resolution / 360.0));
}
double counts_to_angle(int counts, int resolution) {
  return counts * 360.0 / resolution;
}

#ifdef _WIN32
class WindowsSerialStream final : public ByteStream {
 public:
  WindowsSerialStream(const std::string& port, unsigned baudrate, unsigned timeout_ms) {
    std::string path = port.rfind("\\\\.\\", 0) == 0 ? port : "\\\\.\\" + port;
    handle_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0,
                          nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
      throw ProtocolError("cannot open serial port " + port);
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle_, &dcb)) {
      fail("GetCommState failed");
    }
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(handle_, &dcb)) {
      fail("SetCommState failed");
    }
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = timeout_ms;
    timeouts.ReadTotalTimeoutConstant = timeout_ms;
    timeouts.WriteTotalTimeoutConstant = timeout_ms;
    if (!SetCommTimeouts(handle_, &timeouts)) {
      fail("SetCommTimeouts failed");
    }
  }
  ~WindowsSerialStream() override {
    if (handle_ != INVALID_HANDLE_VALUE) {
      CloseHandle(handle_);
    }
  }
  void clear_input() override {
    PurgeComm(handle_, PURGE_RXCLEAR);
  }
  void write(const std::vector<std::uint8_t>& bytes) override {
    DWORD written = 0;
    if (!WriteFile(handle_, bytes.data(), static_cast<DWORD>(bytes.size()), &written, nullptr) ||
        written != bytes.size()) {
      throw ProtocolError("serial write failed");
    }
    FlushFileBuffers(handle_);
  }
  std::vector<std::uint8_t> read_exact(std::size_t size) override {
    std::vector<std::uint8_t> out(size);
    std::size_t offset = 0;
    while (offset < size) {
      DWORD got = 0;
      if (!ReadFile(handle_, out.data() + offset, static_cast<DWORD>(size - offset), &got,
                    nullptr) ||
          got == 0) {
        throw ProtocolError("serial read timeout");
      }
      offset += got;
    }
    return out;
  }

 private:
  void fail(const char* message) {
    CloseHandle(handle_);
    handle_ = INVALID_HANDLE_VALUE;
    throw ProtocolError(message);
  }
  HANDLE handle_{INVALID_HANDLE_VALUE};
};
#endif

std::unique_ptr<ByteStream> open_serial_stream(const std::string& port, unsigned baudrate,
                                               unsigned timeout_ms) {
#ifdef _WIN32
  return std::make_unique<WindowsSerialStream>(port, baudrate, timeout_ms);
#else
  return std::make_unique<PosixSerialStream>(port, baudrate, timeout_ms);
#endif
}

FeetechBus::FeetechBus(std::unique_ptr<ByteStream> stream) : stream_(std::move(stream)) {
  if (!stream_) {
    throw ProtocolError("byte stream is null");
  }
}
StatusPacket FeetechBus::transact(const std::vector<std::uint8_t>& request,
                                  std::uint8_t expected_id) {
  stream_->clear_input();
  stream_->write(request);
  auto header = stream_->read_exact(4);
  auto tail = stream_->read_exact(header.at(3));
  header.insert(header.end(), tail.begin(), tail.end());
  auto status = parse_status_packet(header);
  if (status.servo_id != expected_id) {
    throw ProtocolError("unexpected servo response id");
  }
  return status;
}
bool FeetechBus::ping(std::uint8_t id) {
  try {
    return transact(build_instruction_packet(id, kInstPing), id).error == 0;
  } catch (const ProtocolError&) {
    return false;
  }
}
std::vector<std::uint8_t> FeetechBus::read_register(std::uint8_t id, std::uint8_t address,
                                                    std::uint8_t size) {
  auto status = transact(build_instruction_packet(id, kInstRead, {address, size}), id);
  if (status.error != 0 || status.params.size() != size) {
    throw ProtocolError("register read failed");
  }
  return status.params;
}
void FeetechBus::write_register(std::uint8_t id, std::uint8_t address,
                                const std::vector<std::uint8_t>& data) {
  std::vector<std::uint8_t> params{address};
  params.insert(params.end(), data.begin(), data.end());
  auto status = transact(build_instruction_packet(id, kInstWrite, params), id);
  if (status.error != 0) {
    throw ProtocolError("register write failed");
  }
}
std::uint8_t FeetechBus::read_u8(std::uint8_t id, std::uint8_t address) {
  return read_register(id, address, 1).front();
}
std::uint16_t FeetechBus::read_u16(std::uint8_t id, std::uint8_t address) {
  auto b = read_register(id, address, 2);
  return join_u16(b[0], b[1]);
}
void FeetechBus::write_u8(std::uint8_t id, std::uint8_t address, std::uint8_t value) {
  write_register(id, address, {value});
}
void FeetechBus::write_u16(std::uint8_t id, std::uint8_t address, std::uint16_t value) {
  auto [lo, hi] = split_u16(value);
  write_register(id, address, {lo, hi});
}
std::vector<std::uint8_t> FeetechBus::scan(std::uint8_t first, std::uint8_t last) {
  std::vector<std::uint8_t> ids;
  for (unsigned id = first; id <= last; ++id) {
    if (ping(static_cast<std::uint8_t>(id))) {
      ids.push_back(static_cast<std::uint8_t>(id));
    }
  }
  return ids;
}

}  // namespace diyrobot
