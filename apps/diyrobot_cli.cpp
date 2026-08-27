#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "diyrobot/calibration.hpp"
#include "diyrobot/grasp.hpp"
#include "diyrobot/lerobot_pipeline.hpp"
#include "diyrobot/motion.hpp"
#include "diyrobot/protocol.hpp"
#include "diyrobot/vision.hpp"

namespace {
using namespace diyrobot;
class BusServoIO final : public ServoIO {
 public:
  explicit BusServoIO(FeetechBus& bus) : bus_(bus) {}
  int present_position(std::uint8_t id) override {
    return decode_sign_magnitude(bus_.read_u16(id, reg::present_position));
  }
  std::pair<int, int> position_limits(std::uint8_t id) override {
    return {bus_.read_u16(id, reg::min_position_limit), bus_.read_u16(id, reg::max_position_limit)};
  }
  void goal_position(std::uint8_t id, int value) override {
    bus_.write_u16(id, reg::goal_position, encode_sign_magnitude(value));
  }
  void torque(std::uint8_t id, bool enabled) override {
    bus_.write_u8(id, reg::torque_enable, enabled ? 1 : 0);
  }

 private:
  FeetechBus& bus_;
};
void help() {
  std::cout << "diyRobotCpp - SO-ARM101 C++17 toolkit\n\n"
               "Commands:\n"
               "  protocol-frame                     Print the D1 ping frame\n"
               "  d1-scan [PORT] [--run]             Sequential servo scan (dry-run by default)\n"
               "  d2-home [PORT] [--run] [--confirm-upright]  Move to EEPROM limit midpoints\n"
               "  d3-status [PORT] [--run]           Read position/voltage/temperature\n"
               "  d4-sim                              Run synthetic vision + calibration\n"
               "  d5-record NAMESPACE/DATASET TASK    Print LeRobot record command\n"
               "  d6-train NAMESPACE/DATASET POLICY   Print ACT/diffusion train command\n"
               "  d6-rollout POLICY_PATH TASK         Print a bounded 20-second rollout command\n"
               "  dataset-check PATH                  Validate local LeRobot v3 metadata\n"
               "Hardware commands require --run. Defaults: leader COM22, follower COM24.\n";
}
bool has_run(int argc, char** argv) {
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == "--run") {
      return true;
    }
  }
  return false;
}
bool has_flag(int argc, char** argv, const std::string& flag) {
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == flag) {
      return true;
    }
  }
  return false;
}
std::string port_arg(int argc, char** argv, const std::string& fallback = "COM24") {
  return argc > 2 && std::string(argv[2]).rfind("--", 0) != 0 ? argv[2] : fallback;
}
}  // namespace
int main(int argc, char** argv) {
  using namespace diyrobot;
  try {
    if (argc < 2 || std::string(argv[1]) == "help" || std::string(argv[1]) == "--help") {
      help();
      return 0;
    }
    const std::string command = argv[1];
    if (command == "protocol-frame") {
      for (auto b : build_instruction_packet(1, kInstPing)) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << ' ';
      }
      std::cout << '\n';
      return 0;
    }
    if (command == "d1-scan") {
      const auto port = port_arg(argc, argv);
      if (!has_run(argc, argv)) {
        std::cout << "DRY-RUN: would sequentially ping IDs 1..20 on " << port << " at 1 Mbps\n";
        return 0;
      }
      FeetechBus bus(open_serial_stream(port));
      const auto ids = bus.scan();
      for (auto id : ids) {
        std::cout << "servo " << static_cast<int>(id) << " online\n";
      }
      return ids.empty() ? 2 : 0;
    }
    if (command == "d2-home") {
      const auto port = port_arg(argc, argv);
      if (!has_run(argc, argv) || !has_flag(argc, argv, "--confirm-upright")) {
        std::cout << "DRY-RUN: would read EEPROM limits and trapezoid-move IDs 1..6 to their "
                     "midpoints on "
                  << port
                  << "\nUse both --run and --confirm-upright only after physically placing the arm "
                     "upright.\n";
        return 0;
      }
      FeetechBus bus(open_serial_stream(port));
      BusServoIO io(bus);
      std::map<std::uint8_t, double> midpoints;
      for (std::uint8_t id = 1; id <= 6; ++id) {
        const auto [lo, hi] = io.position_limits(id);
        midpoints[id] = (lo + hi) / 2.0;
      }
      ArmController arm(io, {1, 2, 3, 4, 5, 6}, 30, 25);
      try {
        arm.move_to(midpoints, "trapezoid", 200, 400, 0, true);
      } catch (...) {
        try {
          arm.emergency_stop();
        } catch (const std::exception& stop_error) {
          std::cerr << "emergency stop also failed: " << stop_error.what() << '\n';
        }
        throw;
      }
      return 0;
    }
    if (command == "d3-status") {
      const auto port = port_arg(argc, argv);
      if (!has_run(argc, argv)) {
        std::cout << "DRY-RUN status dashboard: port=" << port << " ids=1..6; no serial writes\n";
        return 0;
      }
      FeetechBus bus(open_serial_stream(port));
      for (std::uint8_t id = 1; id <= 6; ++id) {
        std::cout << "id=" << static_cast<int>(id)
                  << " pos=" << bus.read_u16(id, reg::present_position)
                  << " voltage=" << bus.read_u8(id, reg::present_voltage) / 10.0
                  << "V temp=" << static_cast<int>(bus.read_u8(id, reg::present_temperature))
                  << "C\n";
      }
      return 0;
    }
    if (command == "d4-sim") {
      Image image(640, 480, {240, 240, 240});
      image.fill_rect(380, 220, 40, 40, {0, 0, 255});
      ColorTargetDetector detector({170, 10, 80, 255, 60, 255}, 100, 50000);
      auto target = detector.detect_one(image);
      TableCalibration cal;
      cal.set_homography({{{{0.0002, 0, 0.08}}, {{0, -0.0005, 0.12}}, {{0, 0, 1}}}});
      if (!target) {
        throw std::runtime_error("synthetic target not detected");
      }
      const auto base = cal.pixel_to_base(target->x, target->y);
      std::cout << "target pixel=(" << target->x << ',' << target->y << ") base=(" << base[0] << ','
                << base[1] << ")m\n";
      return 0;
    }
    if (command == "d5-record") {
      if (argc < 4) {
        throw std::invalid_argument("d5-record requires repo id and task");
      }
      RecordSpec spec;
      spec.dataset_repo_id = argv[2];
      spec.task = argv[3];
      std::cout << render_command(spec.command()) << '\n';
      return 0;
    }
    if (command == "d6-train") {
      if (argc < 4) {
        throw std::invalid_argument("d6-train requires repo id and policy");
      }
      TrainSpec spec;
      spec.dataset_repo_id = argv[2];
      spec.policy = argv[3];
      spec.output_dir = "outputs/train/" + spec.policy + "_so101";
      std::cout << render_command(spec.command()) << '\n';
      return 0;
    }
    if (command == "d6-rollout") {
      if (argc < 4) {
        throw std::invalid_argument("d6-rollout requires policy path and task");
      }
      std::cout << render_command(rollout_command(argv[2], argv[3], "COM24", 1, 1280, 720, 30, 20))
                << '\n';
      return 0;
    }
    if (command == "dataset-check") {
      if (argc < 3) {
        throw std::invalid_argument("dataset-check requires a path");
      }
      const auto report = inspect_local_dataset(argv[2]);
      const auto errors = validate_dataset_report(report);
      std::cout << "episodes=" << report.episodes << " frames=" << report.frames
                << " fps=" << report.fps << " cameras=" << report.camera_features.size() << '\n';
      for (const auto& e : errors) {
        std::cout << "FAIL: " << e << '\n';
      }
      return errors.empty() ? 0 : 2;
    }
    throw std::invalid_argument("unknown command: " + command);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}
