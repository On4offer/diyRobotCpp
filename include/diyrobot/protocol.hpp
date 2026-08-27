#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace diyrobot {

constexpr std::uint8_t kBroadcastId = 0xFE;
constexpr std::uint8_t kInstPing = 0x01;
constexpr std::uint8_t kInstRead = 0x02;
constexpr std::uint8_t kInstWrite = 0x03;
constexpr std::uint8_t kInstSyncWrite = 0x83;
constexpr std::uint8_t kErrOverload = 0x20;

namespace reg {
constexpr std::uint8_t min_position_limit = 9;
constexpr std::uint8_t max_position_limit = 11;
constexpr std::uint8_t torque_enable = 40;
constexpr std::uint8_t goal_position = 42;
constexpr std::uint8_t present_position = 56;
constexpr std::uint8_t present_voltage = 62;
constexpr std::uint8_t present_temperature = 63;
}  // namespace reg

class ProtocolError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct StatusPacket {
  std::uint8_t servo_id{};
  std::uint8_t error{};
  std::vector<std::uint8_t> params;
  std::uint8_t length{};
};

std::uint8_t checksum(const std::vector<std::uint8_t>& body);
std::vector<std::uint8_t> build_instruction_packet(int servo_id, int instruction,
                                                   const std::vector<std::uint8_t>& params = {});
StatusPacket parse_status_packet(const std::vector<std::uint8_t>& packet);
std::vector<StatusPacket> parse_status_stream(const std::vector<std::uint8_t>& raw);
std::uint16_t encode_sign_magnitude(int value, unsigned bit = 15);
int decode_sign_magnitude(std::uint16_t value, unsigned bit = 15);
std::pair<std::uint8_t, std::uint8_t> split_u16(std::uint16_t value);
std::uint16_t join_u16(std::uint8_t lo, std::uint8_t hi);
int angle_to_counts(double degrees, int resolution = 4096);
double counts_to_angle(int counts, int resolution = 4096);

class ByteStream {
 public:
  virtual ~ByteStream() = default;
  virtual void clear_input() = 0;
  virtual void write(const std::vector<std::uint8_t>& bytes) = 0;
  virtual std::vector<std::uint8_t> read_exact(std::size_t size) = 0;
};

std::unique_ptr<ByteStream> open_serial_stream(const std::string& port,
                                               unsigned baudrate = 1'000'000,
                                               unsigned timeout_ms = 100);

class FeetechBus {
 public:
  explicit FeetechBus(std::unique_ptr<ByteStream> stream);
  bool ping(std::uint8_t id);
  std::vector<std::uint8_t> read_register(std::uint8_t id, std::uint8_t address, std::uint8_t size);
  void write_register(std::uint8_t id, std::uint8_t address, const std::vector<std::uint8_t>& data);
  std::uint8_t read_u8(std::uint8_t id, std::uint8_t address);
  std::uint16_t read_u16(std::uint8_t id, std::uint8_t address);
  void write_u8(std::uint8_t id, std::uint8_t address, std::uint8_t value);
  void write_u16(std::uint8_t id, std::uint8_t address, std::uint16_t value);
  std::vector<std::uint8_t> scan(std::uint8_t first = 1, std::uint8_t last = 20);

 private:
  StatusPacket transact(const std::vector<std::uint8_t>& request, std::uint8_t expected_id);
  std::unique_ptr<ByteStream> stream_;
};

}  // namespace diyrobot
